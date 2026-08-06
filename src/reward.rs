use crate::env::{
    consts::{ball, car, goal},
    events::team_from_y,
    math::normalized,
    state::{GameState, Player},
};
use rocketsim_rs::sim::Team;

const MAX_REWARDED_BALL_SPEED: f32 = kph_to_vel(110.0);
const MIN_STRONG_TOUCH_SPEED: f32 = kph_to_vel(20.0);
const MAX_STRONG_TOUCH_SPEED: f32 = kph_to_vel(130.0);

// NOTE: THIS FILE IS A DIRECT PORT FROM GIGALEARNCPP! ALL INFO, COMMENTS AND LOGIC REMAINS EQUAL, MINUS SPEED/VELOCITY REWARD!!!
// https://github.com/ZealanL/GigaLearnCPP-Leak/blob/main/GigaLearnCPP/RLGymCPP/src/RLGymCPP/Rewards/CommonRewards.h
pub trait Reward {
    fn reset(&mut self, _agents: &[u32], _initial_state: &GameState) {}

    fn get_rewards(
        &mut self,
        agents: &[u32],
        state: &GameState,
        terminated: bool,
        truncated: bool,
    ) -> Vec<f32>;
}

pub struct WeightedReward {
    reward: Box<dyn Reward>,
    weight: f32,
}

impl WeightedReward {
    #[must_use]
    pub fn new(reward: Box<dyn Reward>, weight: f32) -> Self {
        Self { reward, weight }
    }
}

pub struct CombinedReward {
    rewards: Vec<WeightedReward>,
}

impl CombinedReward {
    #[must_use]
    pub fn new(rewards: Vec<WeightedReward>) -> Self {
        Self { rewards }
    }
}

impl Reward for CombinedReward {
    fn reset(&mut self, agents: &[u32], initial_state: &GameState) {
        for weighted in &mut self.rewards {
            weighted.reward.reset(agents, initial_state);
        }
    }

    fn get_rewards(
        &mut self,
        agents: &[u32],
        state: &GameState,
        terminated: bool,
        truncated: bool,
    ) -> Vec<f32> {
        let mut rewards = vec![0.0; agents.len()];

        for weighted in &mut self.rewards {
            let output = weighted
                .reward
                .get_rewards(agents, state, terminated, truncated);
            assert_eq!(
                output.len(),
                agents.len(),
                "reward output should match agent count"
            );

            for (reward, output) in rewards.iter_mut().zip(output) {
                *reward += output * weighted.weight;
            }
        }

        rewards
    }
}

// This is a wrapper class that makes another reward function zero-sum and team-distributed
// Per-player reward is calculated using: ownReward*(1-teamSpirit) + avgTeamReward*teamSpirit - avgOpponentReward
pub struct ZeroSumReward {
    child: Box<dyn Reward>,
    team_spirit: f32,
    opponent_scale: f32,
}

impl ZeroSumReward {
    #[must_use]
    pub fn new(child: Box<dyn Reward>, team_spirit: f32, opponent_scale: f32) -> Self {
        Self {
            child,
            team_spirit,
            opponent_scale,
        }
    }
}

impl Reward for ZeroSumReward {
    fn reset(&mut self, _agents: &[u32], initial_state: &GameState) {
        let agents = player_ids(initial_state);
        self.child.reset(&agents, initial_state);
    }

    fn get_rewards(
        &mut self,
        agents: &[u32],
        state: &GameState,
        terminated: bool,
        truncated: bool,
    ) -> Vec<f32> {
        let all_agents = player_ids(state);
        let mut rewards = self
            .child
            .get_rewards(&all_agents, state, terminated, truncated);
        assert_eq!(
            rewards.len(),
            state.players.len(),
            "reward output should match player count"
        );

        let mut team_counts = [0_usize; 2];
        let mut team_rewards = [0.0_f32; 2];

        for (player, reward) in state.players.iter().zip(&rewards) {
            let team = team_index(player.team);
            team_counts[team] += 1;
            team_rewards[team] += reward;
        }

        for team in 0..2 {
            team_rewards[team] /= team_counts[team].max(1) as f32;
        }

        for (player, reward) in state.players.iter().zip(&mut rewards) {
            let team = team_index(player.team);
            *reward = *reward * (1.0 - self.team_spirit) + team_rewards[team] * self.team_spirit
                - team_rewards[1 - team] * self.opponent_scale;
        }

        agents
            .iter()
            .map(|&agent| {
                let index = state
                    .players
                    .iter()
                    .position(|player| player.id == agent)
                    .unwrap_or_else(|| panic!("agent {agent} should exist in state"));
                rewards[index]
            })
            .collect()
    }
}

macro_rules! event_reward {
    ($name:ident, $field:ident, $negative:expr) => {
        pub struct $name;

        impl $name {
            #[must_use]
            pub const fn new() -> Self {
                Self
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self::new()
            }
        }

        impl Reward for $name {
            fn get_rewards(
                &mut self,
                agents: &[u32],
                state: &GameState,
                _terminated: bool,
                _truncated: bool,
            ) -> Vec<f32> {
                agent_rewards(agents, state, |player| {
                    let value = f32::from(player.events.$field);
                    if $negative { -value } else { value }
                })
            }
        }
    };
}

event_reward!(PlayerGoalReward, goal, false); // NOTE: Given only to the player who last touched the ball on the opposing team
event_reward!(AssistReward, assist, false);
event_reward!(ShotReward, shot, false);
event_reward!(ShotPassReward, shot_pass, false);
event_reward!(SaveReward, save, false);
event_reward!(BumpReward, bump, false);
event_reward!(BumpedPenalty, bumped, true);
event_reward!(DemoReward, demo, false);
event_reward!(DemoedPenalty, demoed, true);

macro_rules! impl_simple_reward {
    ($name:ident, $reward:expr) => {
        impl $name {
            #[must_use]
            pub const fn new() -> Self {
                Self
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self::new()
            }
        }

        impl Reward for $name {
            fn get_rewards(
                &mut self,
                agents: &[u32],
                state: &GameState,
                _terminated: bool,
                _truncated: bool,
            ) -> Vec<f32> {
                agent_rewards(agents, state, |player| ($reward)(player, state))
            }
        }
    };
}

// Rewards a goal by anyone on the team
// NOTE: Already zero-sum
pub struct GoalReward {
    concede_scale: f32,
}

impl GoalReward {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            concede_scale: -1.0,
        }
    }

    #[must_use]
    pub const fn with_concede_scale(concede_scale: f32) -> Self {
        Self { concede_scale }
    }
}

impl Default for GoalReward {
    fn default() -> Self {
        Self::new()
    }
}

impl Reward for GoalReward {
    fn get_rewards(
        &mut self,
        agents: &[u32],
        state: &GameState,
        _terminated: bool,
        _truncated: bool,
    ) -> Vec<f32> {
        if !state.goal_scored {
            return vec![0.0; agents.len()];
        }

        let conceding_team = team_from_y(state.ball.pos.y);
        agent_rewards(agents, state, |player| {
            if player.team != conceding_team {
                1.0
            } else {
                self.concede_scale
            }
        })
    }
}

// https://github.com/AechPro/rocket-league-gym-sim/blob/main/rlgym_sim/utils/reward_functions/common_rewards/ball_goal_rewards.py
pub struct VelocityBallToGoalReward {
    own_goal: bool,
}

impl VelocityBallToGoalReward {
    #[must_use]
    pub const fn new() -> Self {
        Self { own_goal: false }
    }

    #[must_use]
    pub const fn with_own_goal(own_goal: bool) -> Self {
        Self { own_goal }
    }
}

impl Default for VelocityBallToGoalReward {
    fn default() -> Self {
        Self::new()
    }
}

impl Reward for VelocityBallToGoalReward {
    fn get_rewards(
        &mut self,
        agents: &[u32],
        state: &GameState,
        _terminated: bool,
        _truncated: bool,
    ) -> Vec<f32> {
        agent_rewards(agents, state, |player| {
            let mut target_orange = player.team == Team::Blue;
            if self.own_goal {
                target_orange = !target_orange;
            }

            let target = if target_orange {
                goal::ORANGE_BACK
            } else {
                goal::BLUE_BACK
            };
            let direction = normalized(target - state.ball.pos);
            direction.dot(state.ball.vel / ball::MAX_SPEED)
        })
    }
}

// https://github.com/AechPro/rocket-league-gym-sim/blob/main/rlgym_sim/utils/reward_functions/common_rewards/player_ball_rewards.py
pub struct VelocityPlayerToBallReward;

impl_simple_reward!(
    VelocityPlayerToBallReward,
    |player: &Player, state: &GameState| {
        let direction = normalized(state.ball.pos - player.state.pos);
        direction.dot(player.state.vel / car::MAX_SPEED)
    }
);

// https://github.com/AechPro/rocket-league-gym-sim/blob/main/rlgym_sim/utils/reward_functions/common_rewards/player_ball_rewards.py
pub struct FaceBallReward;

impl_simple_reward!(FaceBallReward, |player: &Player, state: &GameState| {
    let direction = normalized(state.ball.pos - player.state.pos);
    player.state.rot_mat.x_axis.dot(direction)
});

pub struct TouchBallReward;

impl_simple_reward!(TouchBallReward, |player: &Player, _state: &GameState| {
    f32::from(player.ball_touched)
});

pub struct WavedashReward;

impl_simple_reward!(WavedashReward, |player: &Player, state: &GameState| {
    let Some(previous) = previous_player(state, player.id) else {
        return 0.0;
    };

    if player.state.is_on_ground && previous.state.is_flipping && !previous.state.is_on_ground {
        1.0
    } else {
        0.0
    }
});

pub struct PickupBoostReward;

impl_simple_reward!(PickupBoostReward, |player: &Player, state: &GameState| {
    let Some(previous) = previous_player(state, player.id) else {
        return 0.0;
    };

    if player.state.boost > previous.state.boost {
        (player.state.boost / 100.0).sqrt() - (previous.state.boost / 100.0).sqrt()
    } else {
        0.0
    }
});

// https://github.com/AechPro/rocket-league-gym-sim/blob/main/rlgym_sim/utils/reward_functions/common_rewards/misc_rewards.py
pub struct SaveBoostReward {
    exponent: f32,
}

impl SaveBoostReward {
    #[must_use]
    pub const fn new() -> Self {
        Self { exponent: 0.5 }
    }

    #[must_use]
    pub const fn with_exponent(exponent: f32) -> Self {
        Self { exponent }
    }
}

impl Default for SaveBoostReward {
    fn default() -> Self {
        Self::new()
    }
}

impl Reward for SaveBoostReward {
    fn get_rewards(
        &mut self,
        agents: &[u32],
        state: &GameState,
        _terminated: bool,
        _truncated: bool,
    ) -> Vec<f32> {
        agent_rewards(agents, state, |player| {
            (player.state.boost / 100.0)
                .powf(self.exponent)
                .clamp(0.0, 1.0)
        })
    }
}

pub struct AirReward;

impl_simple_reward!(AirReward, |player: &Player, _state: &GameState| {
    f32::from(!player.state.is_on_ground)
});

// Mostly based on the classic Necto rewards
// Total reward output for speeding the ball up to MAX_REWARDED_BALL_SPEED is 1.0
// The bot can do this slowly (putting) or quickly (shooting)
pub struct TouchAccelReward;

impl_simple_reward!(TouchAccelReward, |player: &Player, state: &GameState| {
    let Some(previous) = state.previous.as_deref() else {
        return 0.0;
    };

    if player.ball_touched {
        let previous_speed = (previous.ball.vel.length() / MAX_REWARDED_BALL_SPEED).min(1.0);
        let current_speed = (state.ball.vel.length() / MAX_REWARDED_BALL_SPEED).min(1.0);

        if current_speed > previous_speed {
            current_speed - previous_speed
        } else {
            // Not speeding up the ball so we don't care
            0.0
        }
    } else {
        0.0
    }
});

pub struct StrongTouchReward {
    min_rewarded_speed: f32,
    max_rewarded_speed: f32,
}

impl StrongTouchReward {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            min_rewarded_speed: MIN_STRONG_TOUCH_SPEED,
            max_rewarded_speed: MAX_STRONG_TOUCH_SPEED,
        }
    }

    #[must_use]
    pub const fn with_speeds(min_speed_kph: f32, max_speed_kph: f32) -> Self {
        Self {
            min_rewarded_speed: kph_to_vel(min_speed_kph),
            max_rewarded_speed: kph_to_vel(max_speed_kph),
        }
    }
}

impl Default for StrongTouchReward {
    fn default() -> Self {
        Self::new()
    }
}

impl Reward for StrongTouchReward {
    fn get_rewards(
        &mut self,
        agents: &[u32],
        state: &GameState,
        _terminated: bool,
        _truncated: bool,
    ) -> Vec<f32> {
        agent_rewards(agents, state, |player| {
            let Some(previous) = state.previous.as_deref() else {
                return 0.0;
            };

            if player.ball_touched {
                let hit_force = (state.ball.vel - previous.ball.vel).length();
                if hit_force < self.min_rewarded_speed {
                    return 0.0;
                }

                (hit_force / self.max_rewarded_speed).min(1.0)
            } else {
                0.0
            }
        })
    }
}

fn agent_rewards(
    agents: &[u32],
    state: &GameState,
    mut reward: impl FnMut(&Player) -> f32,
) -> Vec<f32> {
    agents
        .iter()
        .map(|&agent| {
            let player = state
                .player(agent)
                .unwrap_or_else(|| panic!("agent {agent} should exist in state"));
            reward(player)
        })
        .collect()
}

fn previous_player(state: &GameState, id: u32) -> Option<&Player> {
    state.previous.as_deref()?.player(id)
}

fn player_ids(state: &GameState) -> Vec<u32> {
    state.players.iter().map(|player| player.id).collect()
}

const fn kph_to_vel(speed: f32) -> f32 {
    speed * (250.0 / 9.0)
}

const fn team_index(team: Team) -> usize {
    match team {
        Team::Blue => 0,
        Team::Orange => 1,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::env::{
        consts::boost,
        state::{Player, PlayerEvents},
    };
    use rocketsim_rs::{
        glam_ext::{BallA, CarInfoA, glam::Vec3A},
        sim::BoostPadState,
    };

    const TOLERANCE: f32 = 1e-6;

    fn player(id: u32, team: Team) -> Player {
        Player::from(CarInfoA {
            id,
            team,
            ..Default::default()
        })
    }

    fn state(players: Vec<Player>) -> GameState {
        GameState {
            tick_count: 8,
            goal_scored: false,
            ball: BallA::default(),
            players,
            boost_pads: [BoostPadState::default(); boost::NUM_PADS],
            previous: None,
        }
    }

    fn rewards(reward: &mut impl Reward, agents: &[u32], state: &GameState) -> Vec<f32> {
        reward.get_rewards(agents, state, false, false)
    }

    fn assert_close(left: f32, right: f32) {
        assert!((left - right).abs() < TOLERANCE, "{left} != {right}");
    }

    #[test]
    fn event_rewards_match_ggl_flags() {
        let mut blue = player(1, Team::Blue);
        blue.events = PlayerEvents {
            goal: true,
            assist: true,
            shot: true,
            shot_pass: true,
            save: true,
            bump: true,
            bumped: true,
            demo: true,
            demoed: true,
        };
        let state = state(vec![blue]);
        let agents = [1];

        assert_eq!(
            rewards(&mut PlayerGoalReward::new(), &agents, &state),
            [1.0]
        );
        assert_eq!(rewards(&mut AssistReward::new(), &agents, &state), [1.0]);
        assert_eq!(rewards(&mut ShotReward::new(), &agents, &state), [1.0]);
        assert_eq!(rewards(&mut ShotPassReward::new(), &agents, &state), [1.0]);
        assert_eq!(rewards(&mut SaveReward::new(), &agents, &state), [1.0]);
        assert_eq!(rewards(&mut BumpReward::new(), &agents, &state), [1.0]);
        assert_eq!(rewards(&mut BumpedPenalty::new(), &agents, &state), [-1.0]);
        assert_eq!(rewards(&mut DemoReward::new(), &agents, &state), [1.0]);
        assert_eq!(rewards(&mut DemoedPenalty::new(), &agents, &state), [-1.0]);
    }

    #[test]
    fn goal_reward_matches_ggl_teams() {
        let mut state = state(vec![player(1, Team::Blue), player(2, Team::Orange)]);
        state.goal_scored = true;
        state.ball.pos.y = goal::ORANGE_BACK.y;

        assert_eq!(
            rewards(&mut GoalReward::new(), &[1, 2], &state),
            [1.0, -1.0]
        );
    }

    #[test]
    fn directional_rewards_match_ggl_fixture() {
        let mut blue = player(1, Team::Blue);
        blue.state.pos = Vec3A::ZERO;
        blue.state.rot_mat.x_axis = Vec3A::X;
        blue.state.vel = Vec3A::new(car::MAX_SPEED, 0.0, 0.0);
        let mut state = state(vec![blue]);
        state.ball.pos = Vec3A::new(100.0, 0.0, 0.0);

        assert_close(
            rewards(&mut VelocityPlayerToBallReward::new(), &[1], &state)[0],
            1.0,
        );
        assert_close(rewards(&mut FaceBallReward::new(), &[1], &state)[0], 1.0);

        state.ball.pos = Vec3A::new(0.0, 0.0, goal::HEIGHT / 2.0);
        state.ball.vel = Vec3A::new(0.0, ball::MAX_SPEED, 0.0);
        assert_close(
            rewards(&mut VelocityBallToGoalReward::new(), &[1], &state)[0],
            1.0,
        );
        assert_close(
            rewards(
                &mut VelocityBallToGoalReward::with_own_goal(true),
                &[1],
                &state,
            )[0],
            -1.0,
        );
    }

    #[test]
    fn directional_rewards_handle_zero_vectors() {
        let blue = player(1, Team::Blue);
        let mut state = state(vec![blue]);
        state.ball.pos = state.players[0].state.pos;

        assert_eq!(
            rewards(&mut VelocityPlayerToBallReward::new(), &[1], &state),
            [0.0]
        );
        assert_eq!(rewards(&mut FaceBallReward::new(), &[1], &state), [0.0]);

        state.ball.pos = goal::ORANGE_BACK;
        state.ball.vel = Vec3A::Y * ball::MAX_SPEED;
        assert_eq!(
            rewards(&mut VelocityBallToGoalReward::new(), &[1], &state),
            [0.0]
        );
    }

    #[test]
    fn touch_reward_is_binary_per_step() {
        let mut blue = player(1, Team::Blue);
        blue.ball_touched = true;
        let state = state(vec![blue]);

        assert_eq!(rewards(&mut TouchBallReward::new(), &[1], &state), [1.0]);
    }

    #[test]
    fn previous_state_rewards_match_ggl() {
        let mut previous_player = player(1, Team::Blue);
        previous_player.state.boost = 25.0;
        previous_player.state.is_flipping = true;
        previous_player.state.is_on_ground = false;
        let previous = state(vec![previous_player]);

        let mut current_player = player(1, Team::Blue);
        current_player.state.boost = 100.0;
        current_player.state.is_on_ground = true;
        let mut current = state(vec![current_player]);
        current.previous = Some(Box::new(previous));

        assert_eq!(rewards(&mut WavedashReward::new(), &[1], &current), [1.0]);
        assert_close(
            rewards(&mut PickupBoostReward::new(), &[1], &current)[0],
            0.5,
        );
        assert_close(rewards(&mut SaveBoostReward::new(), &[1], &current)[0], 1.0);
        assert_eq!(rewards(&mut AirReward::new(), &[1], &current), [0.0]);
    }

    #[test]
    fn touch_accel_only_rewards_increased_speed() {
        let mut blue = player(1, Team::Blue);
        blue.ball_touched = true;
        let mut previous = state(vec![blue.clone()]);
        previous.ball.vel = Vec3A::new(1000.0, 0.0, 0.0);
        let mut current = state(vec![blue]);
        current.ball.vel = Vec3A::new(2000.0, 0.0, 0.0);
        current.previous = Some(Box::new(previous.clone()));

        assert_close(
            rewards(&mut TouchAccelReward::new(), &[1], &current)[0],
            0.327_272_7,
        );

        current.ball.vel = Vec3A::new(500.0, 0.0, 0.0);
        current.previous = Some(Box::new(previous));
        assert_eq!(rewards(&mut TouchAccelReward::new(), &[1], &current), [0.0]);
    }

    #[test]
    fn strong_touch_uses_full_velocity_change() {
        let mut blue = player(1, Team::Blue);
        blue.ball_touched = true;
        let mut previous = state(vec![blue.clone()]);
        previous.ball.vel = Vec3A::new(1000.0, 0.0, 0.0);
        let mut current = state(vec![blue]);
        current.ball.vel = Vec3A::new(0.0, 1000.0, 0.0);
        current.previous = Some(Box::new(previous));

        assert_close(
            rewards(&mut StrongTouchReward::new(), &[1], &current)[0],
            0.391_628_36,
        );

        current.ball.vel = Vec3A::new(500.0, 0.0, 0.0);
        assert_eq!(
            rewards(&mut StrongTouchReward::new(), &[1], &current),
            [0.0]
        );
    }

    struct IdReward;

    impl Reward for IdReward {
        fn get_rewards(
            &mut self,
            agents: &[u32],
            _state: &GameState,
            _terminated: bool,
            _truncated: bool,
        ) -> Vec<f32> {
            agents
                .iter()
                .map(|agent| match agent {
                    1 => 1.0,
                    2 => 3.0,
                    3 => 2.0,
                    _ => 0.0,
                })
                .collect()
        }
    }

    #[test]
    fn zero_sum_matches_ggl_team_average_formula() {
        let state = state(vec![
            player(1, Team::Blue),
            player(2, Team::Blue),
            player(3, Team::Orange),
        ]);
        let mut reward = ZeroSumReward::new(Box::new(IdReward), 0.5, 1.0);

        assert_eq!(rewards(&mut reward, &[3, 1, 2], &state), [0.0, -0.5, 0.5]);
    }

    #[test]
    fn zero_sum_handles_an_empty_opponent_team() {
        let state = state(vec![player(1, Team::Blue)]);
        let mut reward = ZeroSumReward::new(Box::new(IdReward), 0.5, 1.0);

        assert_eq!(rewards(&mut reward, &[1], &state), [1.0]);
    }

    struct FinalFlagsReward;

    impl Reward for FinalFlagsReward {
        fn get_rewards(
            &mut self,
            agents: &[u32],
            _state: &GameState,
            terminated: bool,
            truncated: bool,
        ) -> Vec<f32> {
            vec![f32::from(terminated) + 2.0 * f32::from(truncated); agents.len()]
        }
    }

    #[test]
    fn combined_reward_preserves_both_final_flags() {
        let state = state(vec![player(1, Team::Blue)]);
        let mut reward =
            CombinedReward::new(vec![WeightedReward::new(Box::new(FinalFlagsReward), 2.0)]);

        assert_eq!(reward.get_rewards(&[1], &state, true, true), [6.0]);
    }
}
