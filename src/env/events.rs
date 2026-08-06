use super::state::{GameState, PlayerEvents};
use rocketsim_rs::sim::{Arena, Team};
use std::{pin::Pin, sync::Mutex};

const SHOT_MIN_SPEED: f32 = 1750.0;
const SHOT_TOUCH_MIN_DELAY: f32 = 0.3;
const SHOT_EVENT_COOLDOWN: f32 = 1.0;
const SHOT_MIN_SCORE_TIME: f32 = 2.0;
const GOAL_MAX_TOUCH_TIME: f32 = 4.0;
const PASS_MAX_TOUCH_TIME: f32 = 2.0;

#[derive(Clone, Copy)]
struct BumpEvent {
    bumper: u32,
    victim: u32,
    is_demo: bool,
}

pub(crate) struct BumpTracker {
    events: Box<Mutex<Vec<BumpEvent>>>,
}

impl BumpTracker {
    pub(crate) fn new() -> Self {
        Self {
            events: Box::new(Mutex::new(Vec::new())),
        }
    }

    pub(crate) fn register(&self, mut arena: Pin<&mut Arena>) {
        let user_data = (&*self.events as *const Mutex<Vec<BumpEvent>>) as usize;
        arena
            .as_mut()
            .set_car_bump_callback(bump_callback, user_data);
    }

    pub(crate) fn clear(&self) {
        if let Ok(mut events) = self.events.lock() {
            events.clear();
        }
    }

    pub(crate) fn apply(&self, state: &mut GameState) {
        let events = if let Ok(mut events) = self.events.lock() {
            std::mem::take(&mut *events)
        } else {
            Vec::new()
        };

        for event in events {
            if let Some(bumper) = state
                .players
                .iter_mut()
                .find(|player| player.id == event.bumper)
            {
                bumper.events.bump = true;
                bumper.events.demo |= event.is_demo;
            }

            if let Some(victim) = state
                .players
                .iter_mut()
                .find(|player| player.id == event.victim)
            {
                victim.events.bumped = true;
                victim.events.demoed |= event.is_demo;
            }
        }
    }
}

fn bump_callback(
    arena: Pin<&mut Arena>,
    bumper: u32,
    victim: u32,
    is_demo: bool,
    user_data: usize,
) {
    if arena.get_car_team(bumper) == arena.get_car_team(victim) {
        return;
    }

    let events = user_data as *const Mutex<Vec<BumpEvent>>;
    if events.is_null() {
        return;
    }

    // pointer is owned by Env and outlives its Arena callback
    let events = unsafe { &*events };
    if let Ok(mut events) = events.lock() {
        events.push(BumpEvent {
            bumper,
            victim,
            is_demo,
        });
    }
}

pub(crate) struct GameEventTracker {
    shot_cooldown: f32,
    ball_shot: bool,
    ball_shot_goal_team: Team,
    ball_scored_last: bool,
}

impl GameEventTracker {
    pub(crate) const fn new() -> Self {
        Self {
            shot_cooldown: 0.0,
            ball_shot: false,
            ball_shot_goal_team: Team::Blue,
            ball_scored_last: false,
        }
    }

    pub(crate) fn reset(&mut self) {
        self.shot_cooldown = 0.0;
        self.ball_shot = false;
        self.ball_scored_last = false;
    }

    pub(crate) fn update<F>(&mut self, state: &mut GameState, tick_rate: f32, mut ball_going_in: F)
    where
        F: FnMut(f32, f32) -> bool,
    {
        let delta_ticks = state
            .previous
            .as_deref()
            .map_or(0, |previous| state.tick_count - previous.tick_count);
        let delta_time = delta_ticks as f32 / tick_rate;
        let scored = state.goal_scored;

        if scored && !self.ball_scored_last {
            let scoring_team = team_from_y(-state.ball.pos.y);
            let (scorer, passer) = find_shooter_passer(
                state,
                scoring_team,
                true,
                seconds_to_ticks(GOAL_MAX_TOUCH_TIME, tick_rate),
                seconds_to_ticks(PASS_MAX_TOUCH_TIME, tick_rate),
            );

            if let Some(scorer) = scorer {
                set_event(state, scorer, |events| events.goal = true);
            }
            if let Some(passer) = passer {
                set_event(state, passer, |events| events.assist = true);
            }
        } else if !self.ball_shot {
            if self.shot_cooldown > 0.0 {
                self.shot_cooldown = (self.shot_cooldown - delta_time).max(0.0);
            } else if state.ball.vel.length_squared() >= SHOT_MIN_SPEED * SHOT_MIN_SPEED
                && ball_going_in(SHOT_MIN_SCORE_TIME, 0.0)
            {
                let goal_team = team_from_y(state.ball.vel.y);
                let shooter_team = opposite_team(goal_team);
                let min_touch_delay = seconds_to_ticks(SHOT_TOUCH_MIN_DELAY, tick_rate);
                let (shooter, passer) = find_shooter_passer(
                    state,
                    shooter_team,
                    true,
                    delta_ticks + min_touch_delay,
                    seconds_to_ticks(PASS_MAX_TOUCH_TIME, tick_rate),
                );

                if let Some(shooter) = shooter {
                    let hit_tick = state
                        .player(shooter)
                        .expect("shot player should exist")
                        .state
                        .ball_hit_info
                        .tick_count_when_hit;

                    if state.tick_count - hit_tick >= min_touch_delay {
                        self.ball_shot = true;
                        self.ball_shot_goal_team = goal_team;
                        self.shot_cooldown = SHOT_EVENT_COOLDOWN;
                        set_event(state, shooter, |events| events.shot = true);
                        if let Some(passer) = passer {
                            set_event(state, passer, |events| events.shot_pass = true);
                        }
                    }
                }
            }
        } else if !ball_going_in(SHOT_MIN_SCORE_TIME, 0.0) {
            let (saver, _) =
                find_shooter_passer(state, self.ball_shot_goal_team, false, delta_ticks, 0);

            if let Some(saver) = saver {
                set_event(state, saver, |events| events.save = true);
            }

            self.ball_shot = false;
        }

        self.ball_scored_last = scored;
    }
}

fn find_shooter_passer(
    state: &GameState,
    team: Team,
    find_passer: bool,
    max_shooter_ticks: u64,
    max_passer_ticks: u64,
) -> (Option<u32>, Option<u32>) {
    let shooter = state
        .players
        .iter()
        .filter(|player| player.team == team && player.state.ball_hit_info.is_valid)
        .filter(|player| {
            player
                .state
                .ball_hit_info
                .tick_count_when_hit
                .saturating_add(max_shooter_ticks)
                >= state.tick_count
        })
        .reduce(|latest, player| {
            if player.state.ball_hit_info.tick_count_when_hit
                > latest.state.ball_hit_info.tick_count_when_hit
            {
                player
            } else {
                latest
            }
        });

    let Some(shooter) = shooter else {
        return (None, None);
    };

    let passer = find_passer.then(|| {
        state
            .players
            .iter()
            .filter(|player| {
                player.team == team
                    && player.id != shooter.id
                    && player.state.ball_hit_info.is_valid
                    && player
                        .state
                        .ball_hit_info
                        .tick_count_when_hit
                        .saturating_add(max_passer_ticks)
                        >= shooter.state.ball_hit_info.tick_count_when_hit
            })
            .reduce(|latest, player| {
                if player.state.ball_hit_info.tick_count_when_hit
                    > latest.state.ball_hit_info.tick_count_when_hit
                {
                    player
                } else {
                    latest
                }
            })
            .map(|player| player.id)
    });

    (Some(shooter.id), passer.flatten())
}

fn set_event(state: &mut GameState, player_id: u32, set: impl FnOnce(&mut PlayerEvents)) {
    if let Some(player) = state
        .players
        .iter_mut()
        .find(|player| player.id == player_id)
    {
        set(&mut player.events);
    }
}

fn seconds_to_ticks(seconds: f32, tick_rate: f32) -> u64 {
    (seconds * tick_rate) as u64
}

pub(crate) const fn team_from_y(y: f32) -> Team {
    if y < 0.0 { Team::Blue } else { Team::Orange }
}

pub(crate) const fn opposite_team(team: Team) -> Team {
    match team {
        Team::Blue => Team::Orange,
        Team::Orange => Team::Blue,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::env::{
        consts::{boost, time},
        state::{Player, boost_pad_indices},
    };
    use rocketsim_rs::{
        glam_ext::{BallA, BallHitInfoA, CarInfoA, CarStateA, glam::Vec3A},
        sim::{BoostPadState, CarConfig},
    };

    fn player(id: u32, team: Team, hit_tick: Option<u64>) -> Player {
        Player::from(CarInfoA {
            id,
            team,
            state: CarStateA {
                ball_hit_info: hit_tick.map_or_else(BallHitInfoA::default, |tick| BallHitInfoA {
                    is_valid: true,
                    tick_count_when_hit: tick,
                    ..Default::default()
                }),
                ..Default::default()
            },
            ..Default::default()
        })
    }

    fn state(tick_count: u64, players: Vec<Player>) -> GameState {
        GameState {
            tick_count,
            goal_scored: false,
            ball: BallA::default(),
            players,
            boost_pads: [BoostPadState::default(); boost::NUM_PADS],
            previous: None,
        }
    }

    #[test]
    fn bump_events_match_ggl_flags() {
        crate::init();

        let mut arena = Arena::default_standard();
        let blue = arena.pin_mut().add_car(Team::Blue, CarConfig::octane());
        let teammate = arena.pin_mut().add_car(Team::Blue, CarConfig::octane());
        let orange = arena.pin_mut().add_car(Team::Orange, CarConfig::octane());
        let pads = boost_pad_indices(&arena);
        let tracker = BumpTracker::new();
        let user_data = (&*tracker.events as *const Mutex<Vec<BumpEvent>>) as usize;

        bump_callback(arena.pin_mut(), blue, teammate, true, user_data);
        bump_callback(arena.pin_mut(), blue, orange, true, user_data);

        let mut state = GameState::from_arena(arena.pin_mut(), &pads);
        tracker.apply(&mut state);

        assert!(state.player(blue).unwrap().events.bump);
        assert!(state.player(blue).unwrap().events.demo);
        assert!(!state.player(teammate).unwrap().events.demoed);
        assert!(state.player(orange).unwrap().events.bumped);
        assert!(state.player(orange).unwrap().events.demoed);
    }

    #[test]
    fn goal_assigns_scorer_and_assist() {
        let mut previous = state(
            92,
            vec![
                player(1, Team::Blue, Some(60)),
                player(2, Team::Blue, Some(50)),
            ],
        );
        previous.ball.pos.y = 5000.0;
        let mut current = previous.clone();
        current.tick_count = 100;
        current.goal_scored = true;
        current.previous = Some(Box::new(previous));
        let mut tracker = GameEventTracker::new();

        tracker.update(&mut current, time::TICK_RATE, |_, _| false);

        assert!(current.player(1).unwrap().events.goal);
        assert!(current.player(2).unwrap().events.assist);
    }

    #[test]
    fn equal_touch_ticks_keep_first_player() {
        let state = state(
            100,
            vec![
                player(1, Team::Blue, Some(60)),
                player(2, Team::Blue, Some(60)),
            ],
        );

        let (shooter, passer) = find_shooter_passer(&state, Team::Blue, true, 480, 240);

        assert_eq!(shooter, Some(1));
        assert_eq!(passer, Some(2));
    }

    #[test]
    fn shot_pass_and_save_follow_ggl_windows() {
        let previous = state(
            92,
            vec![
                player(1, Team::Blue, Some(60)),
                player(2, Team::Blue, Some(50)),
                player(3, Team::Orange, None),
            ],
        );
        let mut shot = previous.clone();
        shot.tick_count = 100;
        shot.ball.vel = Vec3A::new(0.0, 2000.0, 0.0);
        shot.previous = Some(Box::new(previous));
        let mut tracker = GameEventTracker::new();

        tracker.update(&mut shot, time::TICK_RATE, |_, _| true);

        assert!(shot.player(1).unwrap().events.shot);
        assert!(shot.player(2).unwrap().events.shot_pass);

        let mut save = state(
            108,
            vec![
                player(1, Team::Blue, Some(60)),
                player(2, Team::Blue, Some(50)),
                player(3, Team::Orange, Some(104)),
            ],
        );
        save.previous = Some(Box::new(shot));

        tracker.update(&mut save, time::TICK_RATE, |_, _| false);

        assert!(save.player(3).unwrap().events.save);
    }
}
