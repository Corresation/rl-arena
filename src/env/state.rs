use rocketsim_rs::sim::{BallState, CarState};

#[derive(Clone, Copy, Debug)]
pub struct GameState {
    pub tick_count: u64,
    pub ball: BallState,
    pub car: CarState,
}
