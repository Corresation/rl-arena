use super::{
    consts::boost,
    math::{invert_rotation, invert_xy},
};
use rocketsim_rs::{
    glam_ext::{
        BallA, CarInfoA, CarStateA,
        glam::{Mat3A, Vec3A},
    },
    sim::{Arena, BoostPadState},
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

#[derive(Clone, Debug)]
pub struct GameState {
    pub tick_count: u64,
    pub ball: BallA,
    pub players: Vec<CarInfoA>,
    pub boost_pads: [BoostPadState; boost::NUM_PADS],
}

impl GameState {
    pub(crate) fn from_arena(
        mut arena: Pin<&mut Arena>,
        pad_indices: &[usize; boost::NUM_PADS],
    ) -> Self {
        let tick_count = arena.get_tick_count();
        let ball = arena.as_mut().get_ball().to_glam();
        let mut players: Vec<_> = arena
            .as_mut()
            .get_car_infos()
            .into_iter()
            .map(CarInfoA::from)
            .collect();
        // RocketSim stores cars unordered
        players.sort_unstable_by_key(|player| player.id);
        let boost_pads = std::array::from_fn(|index| arena.get_pad_state(pad_indices[index]));

        Self {
            tick_count,
            ball,
            players,
            boost_pads,
        }
    }

    #[must_use]
    pub fn player(&self, id: u32) -> Option<&CarInfoA> {
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
    fn boost_pad_indices_match_real_arena_order() {
        rocketsim_rs::init(None, true);

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
}
