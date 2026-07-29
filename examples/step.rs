use rl_arena::env::Env;

fn main() {
    rocketsim_rs::init(None, true);

    let mut env = Env::new();

    let state = env.step(16);

    println!("position: {}", state.car.pos);
    println!("velocity: {}", state.car.vel);
}
