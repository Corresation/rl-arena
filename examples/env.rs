use rl_arena::env::Env;

fn main() {
    rocketsim_rs::init(None, true);

    let mut env = Env::new();
    env.reset();

    let after = env.step(16);

    println!("position: {}", after.pos);
    println!("velocity: {}", after.vel);
}
