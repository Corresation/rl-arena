mod maps;

pub mod action;
pub mod env;
pub mod episode;
pub mod obs;
pub mod reward;

pub fn init() {
    maps::init();
}
