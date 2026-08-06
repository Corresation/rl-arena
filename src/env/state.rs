use super::{
    consts::boost,
    math::{invert_rotation, invert_xy},
};
use rocketsim_rs::{
    glam_ext::{
        BallA, CarConfigA, CarInfoA, CarStateA,
        glam::{Mat3A, Vec3A},
    },
    sim::{Arena, BoostPadState, Team},
};
use std::pin::Pin;

#[derive(Clone, Copy, Debug)]
pub struct Physics {
    pub pos: Vec3A,
    pub rot_mat: Mat3A,
    pub vel: Vec3A,
    pub ang_vel: Vec3A,
}

impl Physics {
    #[must_use]
    pub const fn from_ball(ball: &BallA) -> Self {
        Self {
            pos: ball.pos,
            rot_mat: ball.rot_mat,
            vel: ball.vel,
            ang_vel: ball.ang_vel,
        }
    }

    #[must_use]
    pub const fn from_car(car: &CarStateA) -> Self {
        Self {
            pos: car.pos,
            rot_mat: car.rot_mat,
            vel: car.vel,
            ang_vel: car.ang_vel,
        }
    }

    #[must_use]
    pub fn inverted(mut self) -> Self {
        self.pos = invert_xy(self.pos);
        self.rot_mat = invert_rotation(self.rot_mat);
        self.vel = invert_xy(self.vel);
        self.ang_vel = invert_xy(self.ang_vel);
        self
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PlayerEvents {
    pub goal: bool,
    pub assist: bool,
    pub shot: bool,
    pub shot_pass: bool,
    pub save: bool,
    pub bump: bool,
    pub bumped: bool,
    pub demo: bool,
    pub demoed: bool,
}

#[derive(Clone, Debug)]
pub struct Player {
    pub id: u32,
    pub team: Team,
    pub state: CarStateA,
    pub config: CarConfigA,
    pub ball_touched: bool,
    pub events: PlayerEvents,
}

impl From<CarInfoA> for Player {
    fn from(player: CarInfoA) -> Self {
        Self {
            id: player.id,
            team: player.team,
            state: player.state,
            config: player.config,
            ball_touched: false,
            events: PlayerEvents::default(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct GameState {
    pub tick_count: u64,
    pub goal_scored: bool,
    pub ball: BallA,
    pub players: Vec<Player>,
    pub boost_pads: [BoostPadState; boost::NUM_PADS],
    pub previous: Option<Box<GameState>>,
}

impl GameState {
    pub(crate) fn from_arena(
        mut arena: Pin<&mut Arena>,
        pad_indices: &[usize; boost::NUM_PADS],
    ) -> Self {
        let tick_count = arena.get_tick_count();
        let goal_scored = arena.is_ball_scored();
        let ball = arena.as_mut().get_ball().to_glam();
        let mut players: Vec<_> = arena
            .as_mut()
            .get_car_infos()
            .into_iter()
            .map(CarInfoA::from)
            .map(Player::from)
            .collect();
        // RocketSim stores cars unordered
        players.sort_unstable_by_key(|player| player.id);
        let boost_pads = std::array::from_fn(|index| arena.get_pad_state(pad_indices[index]));

        Self {
            tick_count,
            goal_scored,
            ball,
            players,
            boost_pads,
            previous: None,
        }
    }

    pub(crate) fn set_previous(&mut self, previous: GameState) {
        for player in &mut self.players {
            let hit = player.state.ball_hit_info;
            player.ball_touched = hit.is_valid && hit.tick_count_when_hit >= previous.tick_count;
        }

        self.previous = Some(Box::new(previous));
    }

    #[must_use]
    pub fn player(&self, id: u32) -> Option<&Player> {
        self.players.iter().find(|player| player.id == id)
    }

    #[must_use]
    pub fn inverted_boost_pads(&self) -> impl ExactSizeIterator<Item = &BoostPadState> {
        self.boost_pads.iter().rev()
    }
}

pub(crate) fn boost_pad_indices(arena: &Arena) -> [usize; boost::NUM_PADS] {
    let pads: Vec<_> = arena.iter_pad_config().collect();

    assert_eq!(
        pads.len(),
        boost::NUM_PADS,
        "standard soccar arena should have {} boost pads",
        boost::NUM_PADS
    );

    std::array::from_fn(|canonical_index| {
        let expected = &boost::PADS[canonical_index];
        let mut matches = pads.iter().enumerate().filter(|(_, pad)| {
            pad.position.x == expected.x
                && pad.position.y == expected.y
                && pad.position.z == expected.z
        });
        let (index, _) = matches
            .next()
            .expect("every canonical boost pad should exist in the arena");

        assert!(
            matches.next().is_none(),
            "canonical boost pad should appear once in the arena"
        );

        index
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn physics_inversion_round_trips() {
        let physics = Physics {
            pos: Vec3A::new(1.0, 2.0, 3.0),
            rot_mat: Mat3A::from_rotation_z(0.7),
            vel: Vec3A::new(4.0, 5.0, 6.0),
            ang_vel: Vec3A::new(7.0, 8.0, 9.0),
        };
        let inverted = physics.inverted();
        let round_trip = inverted.inverted();

        assert_eq!(inverted.pos, Vec3A::new(-1.0, -2.0, 3.0));
        assert_eq!(inverted.vel, Vec3A::new(-4.0, -5.0, 6.0));
        assert_eq!(inverted.ang_vel, Vec3A::new(-7.0, -8.0, 9.0));

        assert_eq!(round_trip.pos, physics.pos);
        assert_eq!(round_trip.rot_mat, physics.rot_mat);
        assert_eq!(round_trip.vel, physics.vel);
        assert_eq!(round_trip.ang_vel, physics.ang_vel);
    }

    #[test]
    fn physics_reads_rocketsim_ball_and_car() {
        let ball_rotation = Mat3A::from_rotation_z(0.2);
        let ball = BallA {
            pos: Vec3A::new(1.0, 2.0, 3.0),
            rot_mat: ball_rotation,
            vel: Vec3A::new(4.0, 5.0, 6.0),
            ang_vel: Vec3A::new(7.0, 8.0, 9.0),
            ..Default::default()
        };
        let car_rotation = Mat3A::from_rotation_z(-0.4);
        let car = CarStateA {
            pos: Vec3A::new(10.0, 11.0, 12.0),
            rot_mat: car_rotation,
            vel: Vec3A::new(13.0, 14.0, 15.0),
            ang_vel: Vec3A::new(16.0, 17.0, 18.0),
            ..Default::default()
        };

        let ball_physics = Physics::from_ball(&ball);
        assert_eq!(ball_physics.pos, ball.pos);
        assert_eq!(ball_physics.rot_mat, ball.rot_mat);
        assert_eq!(ball_physics.vel, ball.vel);
        assert_eq!(ball_physics.ang_vel, ball.ang_vel);

        let car_physics = Physics::from_car(&car);
        assert_eq!(car_physics.pos, car.pos);
        assert_eq!(car_physics.rot_mat, car.rot_mat);
        assert_eq!(car_physics.vel, car.vel);
        assert_eq!(car_physics.ang_vel, car.ang_vel);
    }

    #[test]
    fn per_step_touch_uses_previous_tick_boundary() {
        for (valid, hit_tick, expected) in
            [(true, 100, true), (true, 99, false), (false, 100, false)]
        {
            let player = Player::from(CarInfoA {
                id: 1,
                ..Default::default()
            });
            let previous = GameState {
                tick_count: 100,
                goal_scored: false,
                ball: BallA::default(),
                players: vec![player],
                boost_pads: [BoostPadState::default(); boost::NUM_PADS],
                previous: None,
            };
            let mut current = previous.clone();
            current.tick_count = 108;
            current.players[0].state.ball_hit_info.is_valid = valid;
            current.players[0].state.ball_hit_info.tick_count_when_hit = hit_tick;
            current.set_previous(previous);

            assert_eq!(current.players[0].ball_touched, expected);
        }
    }

    #[test]
    fn boost_pad_indices_match_real_arena_order() {
        crate::init();

        let arena = Arena::default_standard();
        let pad_indices = boost_pad_indices(&arena);
        let arena_pads: Vec<_> = arena.iter_pad_config().collect();
        let mut seen = [false; boost::NUM_PADS];

        for (canonical_index, &arena_index) in pad_indices.iter().enumerate() {
            assert!(!seen[arena_index]);
            seen[arena_index] = true;

            let expected = boost::PADS[canonical_index];
            let actual = arena_pads[arena_index].position;

            assert_eq!(actual.x, expected.x);
            assert_eq!(actual.y, expected.y);
            assert_eq!(actual.z, expected.z);
        }

        assert!(seen.into_iter().all(|was_seen| was_seen));
        assert!(
            pad_indices
                .iter()
                .enumerate()
                .any(|(canonical_index, &arena_index)| canonical_index != arena_index)
        );
    }

    #[test]
    fn goal_state_uses_rocketsim() {
        crate::init();

        let mut arena = Arena::default_standard();
        let pad_indices = boost_pad_indices(&arena);
        let mut ball = arena.pin_mut().get_ball();
        ball.pos.y = crate::env::consts::goal::THRESHOLD_Y + 1.0;
        arena.pin_mut().set_ball(ball);

        let state = GameState::from_arena(arena.pin_mut(), &pad_indices);

        assert!(state.goal_scored);
        assert_eq!(state.goal_scored, arena.is_ball_scored());
    }
}
