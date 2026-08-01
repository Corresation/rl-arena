use rl_arena::env::Env;

fn main() {
    let mut env = Env::new();

    let state = env.step(16);
    let player = state
        .player(env.player_id())
        .expect("controlled player should exist");

    println!("position: {}", player.state.pos);
    println!("velocity: {}", player.state.vel);
}
