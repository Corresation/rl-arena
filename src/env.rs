pub mod consts;
pub mod math;
pub mod state;

pub use state::{GameState, Physics};

use crate::action::{ActionParser, LookupTableAction};
use rocketsim_rs::cxx::UniquePtr;
use rocketsim_rs::sim::{Arena, CarConfig, Team};

pub struct Env {
    arena: UniquePtr<Arena>,
    car_id: u32,
    tick_skip: u32,
    action_parser: LookupTableAction,
    boost_pad_indices: [usize; consts::boost::NUM_PADS],
}

impl Env {
    pub fn new() -> Self {
        crate::init();

        let mut arena = Arena::default_standard();
        let car_id = arena.pin_mut().add_car(Team::Blue, CarConfig::octane());
        let boost_pad_indices = state::boost_pad_indices(&arena);

        let mut env = Self {
            arena,
            car_id,
            tick_skip: 8,
            action_parser: LookupTableAction::new(),
            boost_pad_indices,
        };

        env.reset();
        env
    }

    fn state(&mut self) -> GameState {
        GameState::from_arena(self.arena.pin_mut(), &self.boost_pad_indices)
    }

    pub fn reset(&mut self) -> GameState {
        self.arena.pin_mut().reset_tick_count();
        self.arena.pin_mut().reset_to_random_kickoff(None);

        self.state()
    }

    pub fn action_count(&self) -> usize {
        self.action_parser.action_count()
    }

    pub const fn player_id(&self) -> u32 {
        self.car_id
    }

    pub fn action_mask(&mut self) -> Vec<u8> {
        let car = self.arena.pin_mut().get_car(self.car_id);

        self.action_parser.action_mask(&car)
    }

    pub fn step(&mut self, action_index: usize) -> GameState {
        let controls = self.action_parser.parse_action(action_index);

        self.arena
            .pin_mut()
            .set_car_controls(self.car_id, controls)
            .expect("car should exist");

        self.arena.pin_mut().step(self.tick_skip);

        self.state()
    }
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
        let state = env.reset();

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

        let state = env.reset();

        assert!(state.boost_pads.iter().all(|pad| pad.is_active));
        assert!(state.boost_pads.iter().all(|pad| pad.cooldown == 0.0));
    }

    #[test]
    fn step_uses_tick_skip() {
        let mut env = Env::new();

        for tick_skip in [1, 4, 8, 12, 16] {
            env.tick_skip = tick_skip;
            env.reset();

            let state = env.step(16);

            assert_eq!(state.tick_count, u64::from(tick_skip));
        }
    }

    #[test]
    fn action_moves_car() {
        let mut env = Env::new();
        let player_id = env.player_id();
        let before = env.reset();
        let before = &before
            .player(player_id)
            .expect("controlled player should exist")
            .state;
        let before_speed = before.vel.x * before.vel.x + before.vel.y * before.vel.y;

        let after = env.step(16);
        let after = &after
            .player(player_id)
            .expect("controlled player should exist")
            .state;
        let after_speed = after.vel.x * after.vel.x + after.vel.y * after.vel.y;

        assert!(after_speed > before_speed);
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
}
