use rocketsim_rs::sim::{Arena, CarConfig, CarControls, Team};

fn main() {
    rocketsim_rs::init(None, true);

    for throttle in [1.0, 0.0, -1.0] {
        let mut arena = Arena::default_standard();

        println!("tick rate: {}", arena.get_tick_rate());

        let car_id = arena.pin_mut().add_car(Team::Blue, CarConfig::octane());
        let before = arena.pin_mut().get_car(car_id);

        arena
            .pin_mut()
            .set_car_controls(
                car_id,
                CarControls {
                    throttle,
                    ..Default::default()
                },
            )
            .unwrap();

        let ticks = 60;

        arena.pin_mut().step(ticks);

        println!("stepped {ticks} ticks");

        let after = arena.pin_mut().get_car(car_id);

        println!("throttle: {throttle}");
        println!("car id: {car_id}");
        println!("before position: {}", before.pos);
        println!("before velocity: {}", before.vel);
        println!("after position: {}", after.pos);
        println!("after velocity: {}", after.vel);
        println!();
    }
}
