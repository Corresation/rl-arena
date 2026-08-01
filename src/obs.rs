use crate::env::{
    consts::boost,
    math::to_local,
    state::{GameState, Physics},
};
use rocketsim_rs::{
    glam_ext::{CarInfoA, glam::Vec3A},
    sim::{BoostPadState, CarControls, CarState, Team},
};

const POS_SCALE: f32 = 1.0 / 5000.0;
const VEL_SCALE: f32 = 1.0 / 2300.0;
const ANG_VEL_SCALE: f32 = 1.0 / 3.0;
const BOOST_SCALE: f32 = 1.0 / 100.0;
const BALL_OBS_SIZE: usize = 9;
const ACTION_SIZE: usize = 8;
const PLAYER_OBS_SIZE: usize = 29;
const BASE_OBS_SIZE: usize = BALL_OBS_SIZE + ACTION_SIZE + boost::NUM_PADS;

pub trait ObsBuilder {
    fn reset(&mut self, _agents: &[u32], _initial_state: &GameState) {}

    fn build_obs(&mut self, agents: &[u32], state: &GameState) -> Vec<Vec<f32>>;

    fn obs_size(&self, player_count: usize) -> usize;
}

pub struct AdvancedObs;

impl AdvancedObs {
    #[must_use]
    pub const fn new() -> Self {
        Self
    }

    fn add_vec(obs: &mut Vec<f32>, vector: Vec3A) {
        obs.extend_from_slice(&vector.to_array());
    }

    fn add_physics(obs: &mut Vec<f32>, physics: Physics) {
        Self::add_vec(obs, physics.pos * POS_SCALE);
        Self::add_vec(obs, physics.vel * VEL_SCALE);
        Self::add_vec(obs, physics.ang_vel * ANG_VEL_SCALE);
    }

    fn add_controls(obs: &mut Vec<f32>, controls: CarControls) {
        obs.extend_from_slice(&[
            controls.throttle,
            controls.steer,
            controls.pitch,
            controls.yaw,
            controls.roll,
            f32::from(controls.jump),
            f32::from(controls.boost),
            f32::from(controls.handbrake),
        ]);
    }

    fn pad_value(pad: &BoostPadState) -> f32 {
        if pad.is_active {
            1.0
        } else {
            1.0 / (1.0 + pad.cooldown)
        }
    }

    fn player_obs(
        player: &CarInfoA,
        ball: Physics,
        inverted: bool,
        has_flip_or_jump: bool,
    ) -> Vec<f32> {
        let physics = if inverted {
            Physics::from_car(&player.state).inverted()
        } else {
            Physics::from_car(&player.state)
        };
        let mut obs = Vec::with_capacity(PLAYER_OBS_SIZE);

        Self::add_vec(&mut obs, physics.pos * POS_SCALE);
        Self::add_vec(&mut obs, physics.rot_mat.x_axis);
        Self::add_vec(&mut obs, physics.rot_mat.z_axis);
        Self::add_vec(&mut obs, physics.vel * VEL_SCALE);
        Self::add_vec(&mut obs, physics.ang_vel * ANG_VEL_SCALE);
        Self::add_vec(
            &mut obs,
            to_local(physics.rot_mat, physics.ang_vel) * ANG_VEL_SCALE,
        );
        Self::add_vec(
            &mut obs,
            to_local(physics.rot_mat, ball.pos - physics.pos) * POS_SCALE,
        );
        Self::add_vec(
            &mut obs,
            to_local(physics.rot_mat, ball.vel - physics.vel) * VEL_SCALE,
        );
        obs.extend_from_slice(&[
            player.state.boost * BOOST_SCALE,
            f32::from(player.state.is_on_ground),
            f32::from(has_flip_or_jump),
            f32::from(player.state.is_demoed),
            f32::from(player.state.has_jumped),
        ]);

        debug_assert_eq!(obs.len(), PLAYER_OBS_SIZE);
        obs
    }
}

impl Default for AdvancedObs {
    fn default() -> Self {
        Self::new()
    }
}

impl ObsBuilder for AdvancedObs {
    fn build_obs(&mut self, agents: &[u32], state: &GameState) -> Vec<Vec<f32>> {
        if agents.is_empty() {
            return Vec::new();
        }

        let ball = Physics::from_ball(&state.ball);
        let inverted_ball = ball.inverted();
        let has_flip_or_jump: Vec<_> = state
            .players
            .iter()
            .map(|player| CarState::from(player.state).has_flip_or_jump())
            .collect();
        let player_obs: Vec<_> = state
            .players
            .iter()
            .zip(&has_flip_or_jump)
            .map(|(player, &has_flip)| Self::player_obs(player, ball, false, has_flip))
            .collect();
        let inverted_player_obs: Vec<_> = state
            .players
            .iter()
            .zip(&has_flip_or_jump)
            .map(|(player, &has_flip)| Self::player_obs(player, inverted_ball, true, has_flip))
            .collect();
        let boost_pads: Vec<_> = state.boost_pads.iter().map(Self::pad_value).collect();
        // keep timers paired; ggl swaps its inverted timer array
        let inverted_boost_pads: Vec<_> =
            state.inverted_boost_pads().map(Self::pad_value).collect();

        agents
            .iter()
            .map(|&agent| {
                let player_index = state
                    .players
                    .iter()
                    .position(|player| player.id == agent)
                    .unwrap_or_else(|| panic!("agent {agent} should exist in state"));
                let player = &state.players[player_index];
                let inverted = player.team == Team::Orange;
                let ball = if inverted { inverted_ball } else { ball };
                let boost_pads = if inverted {
                    &inverted_boost_pads
                } else {
                    &boost_pads
                };
                let player_obs = if inverted {
                    &inverted_player_obs
                } else {
                    &player_obs
                };
                let mut obs = Vec::with_capacity(self.obs_size(state.players.len()));

                Self::add_physics(&mut obs, ball);
                Self::add_controls(&mut obs, player.state.last_controls);
                obs.extend_from_slice(boost_pads);
                obs.extend_from_slice(&player_obs[player_index]);

                for (index, other) in state.players.iter().enumerate() {
                    if index != player_index && other.team == player.team {
                        obs.extend_from_slice(&player_obs[index]);
                    }
                }

                for (index, other) in state.players.iter().enumerate() {
                    if other.team != player.team {
                        obs.extend_from_slice(&player_obs[index]);
                    }
                }

                debug_assert_eq!(obs.len(), self.obs_size(state.players.len()));
                obs
            })
            .collect()
    }

    fn obs_size(&self, player_count: usize) -> usize {
        BASE_OBS_SIZE + PLAYER_OBS_SIZE * player_count
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rocketsim_rs::glam_ext::{
        BallA, CarStateA,
        glam::{Mat3A, Vec3A},
    };

    const TOLERANCE: f32 = 1e-6;

    fn assert_close(left: &[f32], right: &[f32]) {
        assert_eq!(left.len(), right.len());

        for (index, (&left, &right)) in left.iter().zip(right).enumerate() {
            assert!(
                (left - right).abs() < TOLERANCE,
                "observation {index} differs: {left} != {right}"
            );
        }
    }

    fn fixture_state() -> GameState {
        let ball = BallA {
            pos: Vec3A::new(5000.0, -10000.0, 2500.0),
            vel: Vec3A::new(2300.0, -4600.0, 1150.0),
            ang_vel: Vec3A::new(3.0, -6.0, 1.5),
            ..Default::default()
        };
        let blue = CarInfoA {
            id: 7,
            team: Team::Blue,
            state: CarStateA {
                pos: Vec3A::new(5000.0, 0.0, 2500.0),
                rot_mat: Mat3A::from_cols(Vec3A::Y, Vec3A::NEG_X, Vec3A::Z),
                vel: Vec3A::new(2300.0, 0.0, 1150.0),
                ang_vel: Vec3A::new(3.0, 6.0, 1.5),
                is_on_ground: false,
                has_jumped: true,
                boost: 50.0,
                last_controls: CarControls {
                    throttle: 0.5,
                    steer: -0.5,
                    pitch: 1.0,
                    yaw: -1.0,
                    roll: 0.25,
                    jump: true,
                    boost: false,
                    handbrake: true,
                },
                ..Default::default()
            },
            ..Default::default()
        };
        let orange = CarInfoA {
            id: 9,
            team: Team::Orange,
            state: CarStateA {
                pos: Vec3A::new(-5000.0, 5000.0, 0.0),
                vel: Vec3A::new(-2300.0, 2300.0, 0.0),
                ang_vel: Vec3A::new(-3.0, 0.0, 3.0),
                boost: 100.0,
                is_demoed: true,
                ..Default::default()
            },
            ..Default::default()
        };
        let mut boost_pads = [BoostPadState {
            is_active: true,
            ..Default::default()
        }; boost::NUM_PADS];
        boost_pads[0] = BoostPadState {
            cooldown: 1.0,
            ..Default::default()
        };
        boost_pads[boost::NUM_PADS - 1] = BoostPadState {
            cooldown: 3.0,
            ..Default::default()
        };

        GameState {
            tick_count: 0,
            ball,
            players: vec![blue, orange],
            boost_pads,
        }
    }

    #[test]
    fn matches_gigalearn_one_v_one_fixture() {
        let state = fixture_state();
        let obs = AdvancedObs::new().build_obs(&[7], &state).remove(0);
        let mut expected = vec![
            1.0, -2.0, 0.5, 1.0, -2.0, 0.5, 1.0, -2.0, 0.5, 0.5, -0.5, 1.0, -1.0, 0.25, 1.0, 0.0,
            1.0, 0.5,
        ];
        expected.extend_from_slice(&[1.0; boost::NUM_PADS - 2]);
        expected.push(0.25);
        expected.extend_from_slice(&[
            1.0, 0.0, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.5, 1.0, 2.0, 0.5, 2.0, -1.0,
            0.5, -2.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0, 1.0,
        ]);
        expected.extend_from_slice(&[
            -1.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 1.0, 0.0, -1.0, 0.0, 1.0, -1.0,
            0.0, 1.0, 2.0, -3.0, 0.5, 2.0, -3.0, 0.5, 1.0, 1.0, 1.0, 1.0, 0.0,
        ]);

        assert_eq!(obs.len(), 109);
        assert_close(&obs, &expected);
    }

    #[test]
    fn orange_view_inverts_ball_and_matching_pad_timers() {
        let state = fixture_state();
        let obs = AdvancedObs::new().build_obs(&[9], &state).remove(0);

        assert_close(
            &obs[..BALL_OBS_SIZE],
            &[-1.0, 2.0, 0.5, -1.0, 2.0, 0.5, -1.0, 2.0, 0.5],
        );
        assert_close(&obs[17..18], &[0.25]);
        assert_close(&obs[50..51], &[0.5]);
        assert_close(&obs[51..54], &[1.0, -1.0, 0.0]);
    }

    #[test]
    fn players_are_grouped_as_self_teammates_opponents() {
        let mut state = fixture_state();
        state.players = [
            (1, Team::Blue),
            (2, Team::Orange),
            (3, Team::Blue),
            (4, Team::Orange),
        ]
        .map(|(id, team)| CarInfoA {
            id,
            team,
            state: CarStateA {
                pos: Vec3A::new(id as f32 * 5000.0, 0.0, 0.0),
                ..Default::default()
            },
            ..Default::default()
        })
        .into();

        let obs = AdvancedObs::new().build_obs(&[4, 3], &state);

        assert_eq!(obs[0].len(), 167);
        assert_close(&obs[0][51..52], &[-4.0]);
        assert_close(&obs[1][51..52], &[3.0]);
        assert_close(&obs[1][80..81], &[1.0]);
        assert_close(&obs[1][109..110], &[2.0]);
        assert_close(&obs[1][138..139], &[4.0]);
    }
}
