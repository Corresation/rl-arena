pub mod consts;
pub mod math;
pub mod state;

pub use state::GameState;

use crate::action::{ActionParser, LookupTableAction};
use rocketsim_rs::cxx::UniquePtr;
use rocketsim_rs::sim::{Arena, CarConfig, Team};

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

    fn state(&mut self) -> GameState {
        GameState {
            tick_count: self.arena.get_tick_count(),
            ball: self.arena.pin_mut().get_ball(),
            car: self.arena.pin_mut().get_car(self.car_id),
        }
    }

    pub fn reset(&mut self) -> GameState {
        self.arena.pin_mut().reset_tick_count();
        self.arena.pin_mut().reset_to_random_kickoff(None);

        self.state()
    }

    pub fn action_count(&self) -> usize {
        self.action_parser.action_count()
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

    #[test]
    fn reset_returns_zero_tick_count() {
        rocketsim_rs::init(None, true);

        let mut env = Env::new();
        let state = env.reset();

        assert_eq!(state.tick_count, 0);
    }

    #[test]
    fn step_uses_tick_skip() {
        rocketsim_rs::init(None, true);

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
        rocketsim_rs::init(None, true);

        let mut env = Env::new();
        let before = env.reset();
        let before_speed =
            before.car.vel.x * before.car.vel.x + before.car.vel.y * before.car.vel.y;

        let after = env.step(16);
        let after_speed = after.car.vel.x * after.car.vel.x + after.car.vel.y * after.car.vel.y;

        assert!(after_speed > before_speed);
    }

    #[test]
    fn exposes_action_space() {
        rocketsim_rs::init(None, true);

        let mut env = Env::new();
        let mask = env.action_mask();

        assert_eq!(env.action_count(), 90);
        assert_eq!(mask.len(), env.action_count());
    }
}
