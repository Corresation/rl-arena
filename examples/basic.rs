use rocketsim_rs::sim::Arena;

fn main() {
    rocketsim_rs::init(None, true);

    let mut arena = Arena::default_standard();
    arena.pin_mut().step(1);

    println!("tick rate: {}", arena.get_tick_rate());
}