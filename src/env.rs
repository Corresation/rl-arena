pub mod consts;
pub(crate) mod events;
pub mod math;
pub mod state;

pub use state::{GameState, Physics};

use crate::action::{ActionParser, LookupTableAction};
use crate::episode::{EpisodeCondition, GoalCondition, TimeoutCondition};
use crate::obs::{AdvancedObs, ObsBuilder};
use crate::reward::{
    AirReward, CombinedReward, FaceBallReward, Reward, TouchBallReward, VelocityPlayerToBallReward,
    WeightedReward,
};
use events::{BumpTracker, GameEventTracker};
use rocketsim_rs::cxx::UniquePtr;
use rocketsim_rs::sim::{Arena, CarConfig, Team};

const TIMEOUT_TICKS: u64 = 300 * consts::time::TICK_RATE as u64;

pub struct StepResult {
    pub state: GameState,
    pub obs: Vec<Vec<f32>>,
    pub rewards: Vec<f32>,
    pub terminated: bool,
    pub truncated: bool,
}

impl StepResult {
    #[must_use]
    pub const fn is_final(&self) -> bool {
        self.terminated || self.truncated
    }
}

pub struct Env {
    arena: UniquePtr<Arena>,
    bump_tracker: BumpTracker,
    event_tracker: GameEventTracker,
    previous_state: Option<GameState>,
    car_id: u32,
    tick_skip: u32,
    action_parser: LookupTableAction,
    obs_builder: AdvancedObs,
    reward: CombinedReward,
    goal_condition: GoalCondition,
    timeout_condition: TimeoutCondition,
    boost_pad_indices: [usize; consts::boost::NUM_PADS],
}

impl Env {
    pub fn new() -> Self {
        crate::init();

        let mut arena = Arena::default_standard();
        let car_id = arena.pin_mut().add_car(Team::Blue, CarConfig::octane());
        let boost_pad_indices = state::boost_pad_indices(&arena);
        let bump_tracker = BumpTracker::new();

        let mut env = Self {
            arena,
            bump_tracker,
            event_tracker: GameEventTracker::new(),
            previous_state: None,
            car_id,
            tick_skip: 8,
            action_parser: LookupTableAction::new(),
            obs_builder: AdvancedObs::new(),
            reward: default_reward(),
            goal_condition: GoalCondition::new(),
            timeout_condition: TimeoutCondition::new(TIMEOUT_TICKS),
            boost_pad_indices,
        };

        env.bump_tracker.register(env.arena.pin_mut());
        env.reset();
        env
    }

    fn state(&mut self) -> GameState {
        GameState::from_arena(self.arena.pin_mut(), &self.boost_pad_indices)
    }

    pub fn reset(&mut self) -> (GameState, Vec<Vec<f32>>) {
        self.bump_tracker.clear();
        self.event_tracker.reset();
        self.previous_state = None;
        self.arena.pin_mut().reset_tick_count();
        self.arena.pin_mut().reset_to_random_kickoff(None);

        let state = self.state();
        let agents = [self.car_id];
        self.previous_state = Some(state.clone());
        self.obs_builder.reset(&agents, &state);
        self.goal_condition.reset(&agents, &state);
        self.timeout_condition.reset(&agents, &state);
        self.reward.reset(&agents, &state);
        let obs = self.obs_builder.build_obs(&agents, &state);

        (state, obs)
    }

    pub fn action_count(&self) -> usize {
        self.action_parser.action_count()
    }

    pub fn obs_size(&self) -> usize {
        self.obs_builder.obs_size(self.arena.num_cars())
    }

    pub const fn player_id(&self) -> u32 {
        self.car_id
    }

    pub fn action_mask(&mut self) -> Vec<u8> {
        let car = self.arena.pin_mut().get_car(self.car_id);

        self.action_parser.action_mask(&car)
    }

    pub fn step(&mut self, action_index: usize) -> StepResult {
        let controls = self.action_parser.parse_action(action_index);

        self.bump_tracker.clear();
        self.arena
            .pin_mut()
            .set_car_controls(self.car_id, controls)
            .expect("car should exist");

        self.arena.pin_mut().step(self.tick_skip);

        let mut state = self.state();
        let previous = self
            .previous_state
            .take()
            .expect("environment should have a previous state");
        state.set_previous(previous);
        self.bump_tracker.apply(&mut state);
        let tick_rate = self.arena.get_tick_rate();
        let arena = &self.arena;
        self.event_tracker
            .update(&mut state, tick_rate, |max_time, extra_margin| {
                arena.is_ball_probably_going_in(Some(max_time), Some(extra_margin))
            });
        let previous = state.previous.take();
        let next_previous = state.clone();
        state.previous = previous;
        self.previous_state = Some(next_previous);

        let agents = [self.car_id];
        let terminated = self.goal_condition.is_done(&agents, &state);
        let truncated = self.timeout_condition.is_done(&agents, &state);
        let rewards = self
            .reward
            .get_rewards(&agents, &state, terminated, truncated);
        let obs = self.obs_builder.build_obs(&agents, &state);

        StepResult {
            state,
            obs,
            rewards,
            terminated,
            truncated,
        }
    }
}

fn default_reward() -> CombinedReward {
    CombinedReward::new(vec![
        WeightedReward::new(Box::new(AirReward::new()), 0.085),
        WeightedReward::new(Box::new(FaceBallReward::new()), 0.1),
        WeightedReward::new(Box::new(VelocityPlayerToBallReward::new()), 1.0),
        WeightedReward::new(Box::new(TouchBallReward::new()), 5.0),
    ])
}

impl Default for Env {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run_soak(seed: u32) -> (GameState, u32) {
        let mut env = Env::new();
        env.arena.pin_mut().reset_tick_count();
        env.arena.pin_mut().reset_to_random_kickoff(Some(seed));

        for step in 0..1_500 {
            env.step((step * 37 + 11) % env.action_count());
        }

        let player_id = env.player_id();
        (env.state(), player_id)
    }

    #[test]
    fn reset_returns_zero_tick_count() {
        let mut env = Env::new();
        let (state, _) = env.reset();

        assert_eq!(state.tick_count, 0);
    }

    #[test]
    fn reset_restores_boost_pads() {
        use rocketsim_rs::sim::BoostPadState;

        let mut env = Env::new();
        env.arena.pin_mut().set_pad_state(
            env.boost_pad_indices[0],
            BoostPadState {
                is_active: false,
                cooldown: 2.0,
                ..Default::default()
            },
        );

        let (state, _) = env.reset();

        assert!(state.boost_pads.iter().all(|pad| pad.is_active));
        assert!(state.boost_pads.iter().all(|pad| pad.cooldown == 0.0));
    }

    #[test]
    fn step_uses_tick_skip() {
        let mut env = Env::new();

        for tick_skip in [1, 4, 8, 12, 16] {
            env.tick_skip = tick_skip;
            env.reset();

            let result = env.step(16);

            assert_eq!(result.state.tick_count, u64::from(tick_skip));
        }
    }

    #[test]
    fn action_moves_car() {
        let mut env = Env::new();
        let player_id = env.player_id();
        let (before, _) = env.reset();
        let before = &before
            .player(player_id)
            .expect("controlled player should exist")
            .state;
        let before_speed = before.vel.x * before.vel.x + before.vel.y * before.vel.y;

        let result = env.step(16);
        let after = &result
            .state
            .player(player_id)
            .expect("controlled player should exist")
            .state;
        let after_speed = after.vel.x * after.vel.x + after.vel.y * after.vel.y;

        assert!(after_speed > before_speed);
    }

    #[test]
    fn reset_and_step_build_observations() {
        let mut env = Env::new();
        let _ = env
            .arena
            .pin_mut()
            .add_car(Team::Orange, CarConfig::octane());

        let (_, initial_obs) = env.reset();

        assert_eq!(initial_obs.len(), 1);
        assert_eq!(initial_obs[0].len(), 109);
        assert_eq!(env.obs_size(), 109);
        assert_eq!(&initial_obs[0][9..17], &[0.0; 8]);

        let result = env.step(16);

        assert_eq!(result.obs.len(), 1);
        assert_eq!(result.obs[0].len(), env.obs_size());
        assert_eq!(result.rewards.len(), 1);
        assert!(result.rewards[0].is_finite());
        let previous = result.state.previous.as_deref().unwrap();
        assert_eq!(previous.tick_count, 0);
        assert!(previous.previous.is_none());
        assert_eq!(
            &result.obs[0][9..17],
            &[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn exposes_action_space() {
        let mut env = Env::new();
        let mask = env.action_mask();

        assert_eq!(env.action_count(), 90);
        assert_eq!(mask.len(), env.action_count());
    }

    #[test]
    fn state_contains_all_players_and_boost_pads() {
        let mut env = Env::new();
        let orange_id = env
            .arena
            .pin_mut()
            .add_car(Team::Orange, CarConfig::octane());
        let state = env.state();

        assert_eq!(state.players.len(), 2);
        assert!(
            state
                .players
                .iter()
                .any(|player| player.id == env.car_id && player.team == Team::Blue)
        );
        assert!(
            state
                .players
                .iter()
                .any(|player| player.id == orange_id && player.team == Team::Orange)
        );
        assert_eq!(state.boost_pads.len(), consts::boost::NUM_PADS);
    }

    #[test]
    fn players_are_sorted_by_id() {
        let mut env = Env::new();
        let orange_id = env
            .arena
            .pin_mut()
            .add_car(Team::Orange, CarConfig::octane());
        let blue_id = env.arena.pin_mut().add_car(Team::Blue, CarConfig::octane());

        let state = env.state();
        let mut expected_ids = [env.car_id, orange_id, blue_id];
        expected_ids.sort_unstable();
        let player_ids: Vec<_> = state.players.iter().map(|player| player.id).collect();

        assert_eq!(player_ids, expected_ids);
        assert_eq!(state.player(orange_id).unwrap().id, orange_id);
        assert!(state.player(u32::MAX).is_none());
    }

    #[test]
    fn inverted_boost_pads_keep_states_paired() {
        use rocketsim_rs::sim::BoostPadState;

        let mut env = Env::new();

        for (canonical_index, &arena_index) in env.boost_pad_indices.iter().enumerate() {
            env.arena.pin_mut().set_pad_state(
                arena_index,
                BoostPadState {
                    is_active: canonical_index.is_multiple_of(2),
                    cooldown: canonical_index as f32,
                    ..Default::default()
                },
            );
        }

        let state = env.state();

        for (index, pad) in state.boost_pads.iter().enumerate() {
            assert_eq!(pad.is_active, index.is_multiple_of(2));
            assert_eq!(pad.cooldown, index as f32);
        }

        for (index, pad) in state.inverted_boost_pads().enumerate() {
            let canonical_index = consts::boost::NUM_PADS - index - 1;

            assert_eq!(pad.is_active, canonical_index.is_multiple_of(2));
            assert_eq!(pad.cooldown, canonical_index as f32);
        }
    }

    #[test]
    fn deterministic_soak_stays_valid() {
        let (first, first_player_id) = run_soak(7);
        let (second, second_player_id) = run_soak(7);

        assert_eq!(first.tick_count, 12_000);
        assert_eq!(second.tick_count, first.tick_count);
        assert!(first.ball.pos.is_finite());
        assert!(first.ball.vel.is_finite());
        assert!(first.ball.ang_vel.is_finite());
        assert!(first.ball.rot_mat.x_axis.is_finite());
        assert!(first.ball.rot_mat.y_axis.is_finite());
        assert!(first.ball.rot_mat.z_axis.is_finite());

        for player in &first.players {
            assert!(player.state.pos.is_finite());
            assert!(player.state.vel.is_finite());
            assert!(player.state.ang_vel.is_finite());
            assert!(player.state.rot_mat.x_axis.is_finite());
            assert!(player.state.rot_mat.y_axis.is_finite());
            assert!(player.state.rot_mat.z_axis.is_finite());
            assert!((0.0..=consts::boost::MAX).contains(&player.state.boost));
        }

        for pad in &first.boost_pads {
            assert!(pad.cooldown.is_finite());
            assert!(pad.cooldown >= 0.0);
        }

        let first_player = first
            .player(first_player_id)
            .expect("controlled player should exist");
        let second_player = second
            .player(second_player_id)
            .expect("controlled player should exist");

        assert_eq!(second.ball.pos, first.ball.pos);
        assert_eq!(second.ball.rot_mat, first.ball.rot_mat);
        assert_eq!(second.ball.vel, first.ball.vel);
        assert_eq!(second.ball.ang_vel, first.ball.ang_vel);
        assert_eq!(second_player.state.pos, first_player.state.pos);
        assert_eq!(second_player.state.rot_mat, first_player.state.rot_mat);
        assert_eq!(second_player.state.vel, first_player.state.vel);
        assert_eq!(second_player.state.ang_vel, first_player.state.ang_vel);
        assert_eq!(second_player.state.boost, first_player.state.boost);

        for (left, right) in first.boost_pads.iter().zip(&second.boost_pads) {
            assert_eq!(right.is_active, left.is_active);
            assert_eq!(right.cooldown, left.cooldown);
        }
    }

    #[test]
    fn step_preserves_simultaneous_final_flags() {
        let mut env = Env::new();
        env.timeout_condition = TimeoutCondition::new(u64::from(env.tick_skip));
        env.reset();

        let mut ball = env.arena.pin_mut().get_ball();
        ball.pos.y = consts::goal::THRESHOLD_Y + 1.0;
        env.arena.pin_mut().set_ball(ball);

        let result = env.step(16);

        assert!(result.terminated);
        assert!(result.truncated);
        assert!(result.is_final());
    }

    #[test]
    fn final_step_does_not_reset_environment() {
        let mut env = Env::new();
        env.timeout_condition = TimeoutCondition::new(u64::from(env.tick_skip));
        env.reset();

        let first = env.step(16);
        let second = env.step(16);

        assert!(first.truncated);
        assert_eq!(first.state.tick_count, u64::from(env.tick_skip));
        assert_eq!(second.state.tick_count, u64::from(env.tick_skip * 2));
    }
}
