use rl_arena::env::Env;

fn main() {
    let mut env = Env::new();

    let result = env.step(16);
    let player = result
        .state
        .player(env.player_id())
        .expect("controlled player should exist");

    println!("position: {}", player.state.pos);
    println!("velocity: {}", player.state.vel);
    println!("observation size: {}", result.obs[0].len());
}
