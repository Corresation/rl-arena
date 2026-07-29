use rl_arena::env::Env;
use rocketsim_rs::sim::CarControls;

fn main() {
    rocketsim_rs::init(None, true);

    let mut env = Env::new();
    env.reset();

    let after = env.step(CarControls {
        throttle: 1.0,
        ..Default::default()
    });

    println!("position: {}", after.pos);
    println!("velocity: {}", after.vel);
}
