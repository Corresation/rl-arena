use crate::action::{ActionParser, LookupTableAction};
use rocketsim_rs::cxx::UniquePtr;
use rocketsim_rs::sim::{Arena, CarConfig, CarState, Team};

pub struct Env {
    arena: UniquePtr<Arena>,
    car_id: u32,
    tick_skip: u32,
    action_parser: LookupTableAction,
}

impl Env {
    pub fn new() -> Self {
        let mut arena = Arena::default_standard();
        let car_id = arena.pin_mut().add_car(Team::Blue, CarConfig::octane());

        let mut env = Self {
            arena,
            car_id,
            tick_skip: 8,
            action_parser: LookupTableAction::new(),
        };

        env.reset();
        env
    }

    pub fn reset(&mut self) -> CarState {
        self.arena.pin_mut().reset_tick_count();
        self.arena.pin_mut().reset_to_random_kickoff(None);

        self.arena.pin_mut().get_car(self.car_id)
    }

    pub fn step(&mut self, action_index: usize) -> CarState {
        let controls = self.action_parser.parse_action(action_index);

        self.arena
            .pin_mut()
            .set_car_controls(self.car_id, controls)
            .expect("car should exist");

        self.arena.pin_mut().step(self.tick_skip);

        self.arena.pin_mut().get_car(self.car_id)
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

    #[test]
    fn step_and_reset() {
        rocketsim_rs::init(None, true);

        let mut env = Env::new();
        let before = env.reset();
        let before_speed = before.vel.x * before.vel.x + before.vel.y * before.vel.y;

        let after = env.step(16);
        let after_speed = after.vel.x * after.vel.x + after.vel.y * after.vel.y;

        assert!(after_speed > before_speed);
        assert_eq!(env.arena.get_tick_count(), u64::from(env.tick_skip));

        env.reset();

        assert_eq!(env.arena.get_tick_count(), 0);
    }
}
