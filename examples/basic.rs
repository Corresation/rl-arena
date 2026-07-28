use rocketsim_rs::sim::{Arena, CarConfig, Team};

fn main() {
    rocketsim_rs::init(None, true);

    let mut arena = Arena::default_standard();

    println!("tick rate: {}", arena.get_tick_rate());

    let car_id = arena.pin_mut().add_car(Team::Blue, CarConfig::octane());
    let before = arena.pin_mut().get_car(car_id);

    let ticks = 120;

    arena.pin_mut().step(ticks);

    println!("stepped {ticks} ticks");

    let after = arena.pin_mut().get_car(car_id);

    println!("car id: {car_id}");
    println!("before: {}", before.pos);
    println!("after: {}", after.pos);
}
