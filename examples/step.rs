use rl_arena::env::Env;

fn main() {
    let mut env = Env::new();

    let (state, obs) = env.step(16);
    let player = state
        .player(env.player_id())
        .expect("controlled player should exist");

    println!("position: {}", player.state.pos);
    println!("velocity: {}", player.state.vel);
    println!("observation size: {}", obs[0].len());
}
