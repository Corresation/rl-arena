use rocketsim_rs::sim::{CarControls, CarState};

const ANALOG_INPUTS: [f32; 3] = [-1.0, 0.0, 1.0];
const BOOLEAN_INPUTS: [bool; 2] = [false, true];

pub trait ActionParser {
    fn parse_action(&self, index: usize) -> CarControls;

    fn action_count(&self) -> usize;

    fn action_mask(&self, _car: &CarState) -> Vec<u8> {
        vec![1; self.action_count()]
    }
}

pub struct LookupTableAction {
    actions: Vec<CarControls>,
    ground_mask: Vec<u8>,
    air_mask: Vec<u8>,
    jump_mask: Vec<u8>,
    boost_mask: Vec<u8>,
}

impl LookupTableAction {
    pub fn new() -> Self {
        let (actions, num_ground_actions) = Self::make_lookup_table();
        let mut ground_mask = vec![0; actions.len()];
        let mut air_mask = vec![0; actions.len()];
        let mut jump_mask = vec![0; actions.len()];
        let mut boost_mask = vec![0; actions.len()];

        for (index, action) in actions.iter().enumerate() {
            jump_mask[index] = u8::from(action.jump);
            boost_mask[index] = u8::from(action.boost);
            ground_mask[index] = u8::from(index < num_ground_actions);
            // fixed the first aerial action being skipped
            air_mask[index] = u8::from(index >= num_ground_actions && !action.jump);

            if index < num_ground_actions {
                let boost = if action.boost { 1.0 } else { 0.0 };

                if action.throttle == boost && (action.yaw != 0.0) == action.handbrake {
                    air_mask[index] = 1;
                }
            }
        }

        Self {
            actions,
            ground_mask,
            air_mask,
            jump_mask,
            boost_mask,
        }
    }

    fn make_lookup_table() -> (Vec<CarControls>, usize) {
        let mut actions = Vec::with_capacity(90);

        for throttle in ANALOG_INPUTS {
            for steer in ANALOG_INPUTS {
                for boost in BOOLEAN_INPUTS {
                    for handbrake in BOOLEAN_INPUTS {
                        if boost && throttle != 1.0 {
                            continue;
                        }

                        actions.push(CarControls {
                            throttle,
                            steer,
                            yaw: steer,
                            boost,
                            handbrake,
                            ..Default::default()
                        });
                    }
                }
            }
        }

        let num_ground_actions = actions.len();

        for pitch in ANALOG_INPUTS {
            for yaw in ANALOG_INPUTS {
                for roll in ANALOG_INPUTS {
                    for jump in BOOLEAN_INPUTS {
                        for boost in BOOLEAN_INPUTS {
                            if jump && yaw != 0.0 {
                                continue;
                            }

                            if pitch == 0.0 && roll == 0.0 && !jump {
                                continue;
                            }

                            let handbrake = jump && (pitch != 0.0 || yaw != 0.0 || roll != 0.0);

                            actions.push(CarControls {
                                throttle: if boost { 1.0 } else { 0.0 },
                                steer: yaw,
                                pitch,
                                yaw,
                                roll,
                                jump,
                                boost,
                                handbrake,
                            });
                        }
                    }
                }
            }
        }

        (actions, num_ground_actions)
    }

    fn build_mask(&self, is_on_ground: bool, has_boost: bool, can_jump: bool) -> Vec<u8> {
        let mut result = if is_on_ground {
            self.ground_mask.clone()
        } else {
            self.air_mask.clone()
        };

        if can_jump {
            for (allowed, jump) in result.iter_mut().zip(&self.jump_mask) {
                *allowed |= *jump;
            }
        }

        if !has_boost {
            // fixed zero-boost jump actions being re-enabled
            for (allowed, boost) in result.iter_mut().zip(&self.boost_mask) {
                if *boost != 0 {
                    *allowed = 0;
                }
            }
        }

        result
    }
}

impl Default for LookupTableAction {
    fn default() -> Self {
        Self::new()
    }
}

impl ActionParser for LookupTableAction {
    fn parse_action(&self, index: usize) -> CarControls {
        assert!(
            index < self.actions.len(),
            "action index {index} out of range ({} actions)",
            self.actions.len()
        );

        self.actions[index]
    }

    fn action_count(&self) -> usize {
        self.actions.len()
    }

    fn action_mask(&self, car: &CarState) -> Vec<u8> {
        let is_turtled = car.world_contact.has_contact && car.world_contact.contact_normal.z > 0.9;

        self.build_mask(
            car.is_on_ground,
            car.boost != 0.0,
            car.has_flip_or_jump() || is_turtled,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lookup_table_has_expected_shape() {
        let parser = LookupTableAction::new();

        assert_eq!(parser.action_count(), 90);
        assert!(parser.ground_mask[..24].iter().all(|allowed| *allowed == 1));
        assert!(parser.ground_mask[24..].iter().all(|allowed| *allowed == 0));

        for action in &parser.actions[..24] {
            assert_eq!(action.pitch, 0.0);
            assert_eq!(action.yaw, action.steer);
            assert_eq!(action.roll, 0.0);
            assert!(!action.jump);
            assert!(!action.boost || action.throttle == 1.0);
        }

        for action in &parser.actions[24..] {
            assert_eq!(action.steer, action.yaw);
            assert_eq!(action.throttle, if action.boost { 1.0 } else { 0.0 });
            assert!(!action.jump || action.yaw == 0.0);
            assert_eq!(
                action.handbrake,
                action.jump && (action.pitch != 0.0 || action.yaw != 0.0 || action.roll != 0.0)
            );
        }
    }

    #[test]
    fn action_16_is_straight_forward_throttle() {
        let action = LookupTableAction::new().parse_action(16);

        assert_eq!(action.throttle, 1.0);
        assert_eq!(action.steer, 0.0);
        assert_eq!(action.pitch, 0.0);
        assert_eq!(action.yaw, 0.0);
        assert_eq!(action.roll, 0.0);
        assert!(!action.jump);
        assert!(!action.boost);
        assert!(!action.handbrake);
    }

    #[test]
    fn first_aerial_action_is_available_in_air() {
        let parser = LookupTableAction::new();

        assert_eq!(parser.air_mask[24], 1);
    }

    #[test]
    fn zero_boost_never_allows_boost_actions() {
        let parser = LookupTableAction::new();
        let mask = parser.build_mask(false, false, true);

        for (allowed, action) in mask.iter().zip(&parser.actions) {
            if action.boost {
                assert_eq!(*allowed, 0);
            }
        }
    }

    #[test]
    fn action_mask_uses_rocketsim_car_state() {
        let parser = LookupTableAction::new();
        let mut car = CarState {
            is_on_ground: false,
            has_flipped: true,
            has_double_jumped: true,
            boost: 0.0,
            ..Default::default()
        };

        let mask = parser.action_mask(&car);
        for (allowed, action) in mask.iter().zip(&parser.actions) {
            if action.jump || action.boost {
                assert_eq!(*allowed, 0);
            }
        }

        car.world_contact.has_contact = true;
        car.world_contact.contact_normal.z = 1.0;

        let mask = parser.action_mask(&car);
        for (allowed, action) in mask.iter().zip(&parser.actions) {
            if action.boost {
                assert_eq!(*allowed, 0);
            } else if action.jump {
                assert_eq!(*allowed, 1);
            }
        }
    }

    #[test]
    fn masks_match_action_count() {
        let parser = LookupTableAction::new();
        let count = parser.action_count();

        assert_eq!(parser.ground_mask.len(), count);
        assert_eq!(parser.air_mask.len(), count);
        assert_eq!(parser.jump_mask.len(), count);
        assert_eq!(parser.boost_mask.len(), count);
    }

    #[test]
    #[should_panic(expected = "action index 90")]
    fn invalid_action_index_has_clear_message() {
        LookupTableAction::new().parse_action(90);
    }
}
