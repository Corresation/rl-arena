pub mod field {
    pub use rocketsim_rs::consts::{
        ARENA_EXTENT_X as SIDE_WALL_X, ARENA_EXTENT_Y as BACK_WALL_Y, ARENA_HEIGHT as MESH_HEIGHT,
        GRAVITY_Z,
    };

    // ceiling change; RocketSim mesh is 2048
    pub const CEILING_Z: f32 = 2044.0;
    pub const BACK_NET_Y: f32 = 6000.0;
    pub const CORNER_CATHETUS_LENGTH: f32 = 1152.0;
    pub const RAMP_HEIGHT: f32 = 256.0;
}

pub mod goal {
    use super::ball;

    pub use rocketsim_rs::consts::SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y as SCORE_BASE_THRESHOLD_Y;

    pub const HEIGHT: f32 = 642.775;
    pub const CENTER_TO_POST: f32 = 892.755;
    pub const THRESHOLD_Y: f32 = SCORE_BASE_THRESHOLD_Y + ball::RADIUS;
}

pub mod ball {
    pub use rocketsim_rs::consts::{
        BALL_COLLISION_RADIUS_SOCCAR as RADIUS, // hi Zealan I changed it from 92.75 ;-)
        BALL_MASS_BT as MASS,
        BALL_MAX_ANG_SPEED as MAX_ANG_SPEED,
        BALL_MAX_SPEED as MAX_SPEED,
        BALL_REST_Z as RESTING_HEIGHT,
    };
}

pub mod car {
    pub use rocketsim_rs::consts::{
        CAR_MASS_BT as MASS, CAR_MAX_ANG_SPEED as MAX_ANG_SPEED, CAR_MAX_SPEED as MAX_SPEED,
        DEMO_RESPAWN_TIME,
    };

    pub mod supersonic {
        pub use rocketsim_rs::consts::{
            SUPERSONIC_MAINTAIN_MAX_TIME as MAINTAIN_MAX_TIME,
            SUPERSONIC_MAINTAIN_MIN_SPEED as MAINTAIN_MIN_SPEED,
            SUPERSONIC_START_SPEED as START_SPEED,
        };
    }

    pub mod drive {
        pub use rocketsim_rs::consts::{
            BRAKE_TORQUE_AMOUNT, BRAKING_NO_THROTTLE_SPEED_THRESH, COASTING_BRAKE_FACTOR,
            POWERSLIDE_FALL_RATE, POWERSLIDE_RISE_RATE, STOPPING_FORWARD_VEL, THROTTLE_AIR_ACCEL,
            THROTTLE_DEADZONE, THROTTLE_TORQUE_AMOUNT,
        };
    }

    pub mod jump {
        pub use rocketsim_rs::consts::{
            DOUBLEJUMP_MAX_DELAY, JUMP_ACCEL as ACCEL, JUMP_IMMEDIATE_FORCE as IMMEDIATE_FORCE,
            JUMP_MAX_TIME as MAX_TIME, JUMP_MIN_TIME as MIN_TIME,
            JUMP_RESET_TIME_PAD as RESET_TIME_PAD,
        };
    }
}

pub mod boost {
    use rocketsim_rs::math::Vec3;

    pub use rocketsim_rs::consts::boostpads::{
        BOOST_AMOUNT_BIG as BIG_PAD_REFILL, BOOST_AMOUNT_SMALL as SMALL_PAD_REFILL,
        COOLDOWN_BIG as BIG_PAD_RECHARGE_TIME, COOLDOWN_SMALL as SMALL_PAD_RECHARGE_TIME,
    };
    pub use rocketsim_rs::consts::{
        BOOST_ACCEL_AIR as ACCEL_AIR, BOOST_ACCEL_GROUND as ACCEL_GROUND, BOOST_MAX as MAX,
        BOOST_MIN_TIME as MIN_TIME, BOOST_SPAWN_AMOUNT as SPAWN_AMOUNT,
        BOOST_USED_PER_SECOND as CONSUMED_PER_SECOND,
    };

    // patched RLGym/GGL 3310 pad typo
    pub const PADS: [Vec3; 34] = [
        Vec3::new(0.0, -4240.0, 70.0),
        Vec3::new(-1792.0, -4184.0, 70.0),
        Vec3::new(1792.0, -4184.0, 70.0),
        Vec3::new(-3072.0, -4096.0, 73.0),
        Vec3::new(3072.0, -4096.0, 73.0),
        Vec3::new(-940.0, -3308.0, 70.0),
        Vec3::new(940.0, -3308.0, 70.0),
        Vec3::new(0.0, -2816.0, 70.0),
        Vec3::new(-3584.0, -2484.0, 70.0),
        Vec3::new(3584.0, -2484.0, 70.0),
        Vec3::new(-1788.0, -2300.0, 70.0),
        Vec3::new(1788.0, -2300.0, 70.0),
        Vec3::new(-2048.0, -1036.0, 70.0),
        Vec3::new(0.0, -1024.0, 70.0),
        Vec3::new(2048.0, -1036.0, 70.0),
        Vec3::new(-3584.0, 0.0, 73.0),
        Vec3::new(-1024.0, 0.0, 70.0),
        Vec3::new(1024.0, 0.0, 70.0),
        Vec3::new(3584.0, 0.0, 73.0),
        Vec3::new(-2048.0, 1036.0, 70.0),
        Vec3::new(0.0, 1024.0, 70.0),
        Vec3::new(2048.0, 1036.0, 70.0),
        Vec3::new(-1788.0, 2300.0, 70.0),
        Vec3::new(1788.0, 2300.0, 70.0),
        Vec3::new(-3584.0, 2484.0, 70.0),
        Vec3::new(3584.0, 2484.0, 70.0),
        Vec3::new(0.0, 2816.0, 70.0),
        Vec3::new(-940.0, 3308.0, 70.0),
        Vec3::new(940.0, 3308.0, 70.0),
        Vec3::new(-3072.0, 4096.0, 73.0),
        Vec3::new(3072.0, 4096.0, 73.0),
        Vec3::new(-1792.0, 4184.0, 70.0),
        Vec3::new(1792.0, 4184.0, 70.0),
        Vec3::new(0.0, 4240.0, 70.0),
    ];

    pub const NUM_PADS: usize = PADS.len();
}

pub mod time {
    pub const TICK_RATE: f32 = 120.0;
    pub const TICK_TIME: f32 = 1.0 / TICK_RATE;
}

#[cfg(test)]
mod tests {
    use super::{ball, boost, goal, time};
    use rocketsim_rs::{consts::boostpads, math::Vec3, sim::Arena};

    const BIG_PAD_INDICES: [usize; 6] = [3, 4, 15, 18, 29, 30];

    fn same_position(left: &Vec3, right: &Vec3) -> bool {
        left.x == right.x && left.y == right.y && left.z == right.z
    }

    fn source_match_count(pad: &Vec3) -> usize {
        boostpads::LOCS_SMALL_SOCCAR
            .iter()
            .chain(&boostpads::LOCS_BIG_SOCCAR)
            .filter(|source| same_position(pad, source))
            .count()
    }

    #[test]
    fn boost_pad_count_is_34() {
        assert_eq!(boost::NUM_PADS, 34);
        assert_eq!(boost::PADS.len(), boost::NUM_PADS);
    }

    #[test]
    fn boost_pads_are_mirrored() {
        for (pad, mirrored) in boost::PADS.iter().zip(boost::PADS.iter().rev()) {
            assert_eq!(pad.x, -mirrored.x);
            assert_eq!(pad.y, -mirrored.y);
            assert_eq!(pad.z, mirrored.z);
        }
    }

    #[test]
    fn boost_pads_do_not_use_3310() {
        assert!(boost::PADS.iter().all(|pad| pad.y.abs() != 3310.0));
    }

    #[test]
    fn boost_pads_match_rocketsim_once() {
        for source in boostpads::LOCS_SMALL_SOCCAR
            .iter()
            .chain(&boostpads::LOCS_BIG_SOCCAR)
        {
            assert_eq!(
                boost::PADS
                    .iter()
                    .filter(|pad| same_position(pad, source))
                    .count(),
                1
            );
        }

        for pad in &boost::PADS {
            assert_eq!(source_match_count(pad), 1);
        }
    }

    #[test]
    fn large_boost_pad_indices_match_rocketsim() {
        for (index, pad) in boost::PADS.iter().enumerate() {
            let is_large = boostpads::LOCS_BIG_SOCCAR
                .iter()
                .any(|source| same_position(pad, source));

            assert_eq!(is_large, BIG_PAD_INDICES.contains(&index));
        }
    }

    #[test]
    fn goal_threshold_uses_ball_radius() {
        assert_eq!(
            goal::THRESHOLD_Y,
            goal::SCORE_BASE_THRESHOLD_Y + ball::RADIUS
        );
        assert_eq!(goal::THRESHOLD_Y, 5215.5);
    }

    #[test]
    fn standard_arena_uses_tick_rate_constant() {
        crate::init();

        let arena = Arena::default_standard();
        let tick_rate = arena.get_tick_rate();

        assert!((tick_rate - time::TICK_RATE).abs() < 0.001);
        assert!((time::TICK_TIME - 1.0 / tick_rate).abs() < f32::EPSILON);
    }
}
