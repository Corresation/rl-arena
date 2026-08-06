use crate::env::GameState;

pub trait EpisodeCondition {
    fn reset(&mut self, _agents: &[u32], _initial_state: &GameState) {}

    fn is_done(&mut self, agents: &[u32], state: &GameState) -> bool;
}

pub struct GoalCondition;

impl GoalCondition {
    #[must_use]
    pub const fn new() -> Self {
        Self
    }
}

impl Default for GoalCondition {
    fn default() -> Self {
        Self::new()
    }
}

impl EpisodeCondition for GoalCondition {
    fn is_done(&mut self, _agents: &[u32], state: &GameState) -> bool {
        state.goal_scored
    }
}

pub struct TimeoutCondition {
    start_tick: u64,
    max_ticks: u64,
}

impl TimeoutCondition {
    #[must_use]
    pub const fn new(max_ticks: u64) -> Self {
        Self {
            start_tick: 0,
            max_ticks,
        }
    }
}

impl EpisodeCondition for TimeoutCondition {
    fn reset(&mut self, _agents: &[u32], initial_state: &GameState) {
        self.start_tick = initial_state.tick_count;
    }

    fn is_done(&mut self, _agents: &[u32], state: &GameState) -> bool {
        state.tick_count - self.start_tick >= self.max_ticks
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::env::consts::boost;
    use rocketsim_rs::{glam_ext::BallA, sim::BoostPadState};

    fn state(tick_count: u64, goal_scored: bool) -> GameState {
        GameState {
            tick_count,
            goal_scored,
            ball: BallA::default(),
            players: Vec::new(),
            boost_pads: [BoostPadState::default(); boost::NUM_PADS],
            previous: None,
        }
    }

    #[test]
    fn goal_uses_state_result() {
        let mut condition = GoalCondition::new();

        assert!(!condition.is_done(&[], &state(0, false)));
        assert!(condition.is_done(&[], &state(0, true)));
    }

    #[test]
    fn timeout_uses_ticks_since_reset() {
        let mut condition = TimeoutCondition::new(60);
        condition.reset(&[], &state(100, false));

        assert!(!condition.is_done(&[], &state(159, false)));
        assert!(condition.is_done(&[], &state(160, false)));

        condition.reset(&[], &state(500, false));

        assert!(!condition.is_done(&[], &state(559, false)));
        assert!(condition.is_done(&[], &state(560, false)));
    }
}
