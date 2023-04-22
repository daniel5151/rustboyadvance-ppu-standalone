use std::cmp::Ordering;
use std::collections::BinaryHeap;

use serde::{Deserialize, Serialize};

const NUM_EVENTS: usize = 32;

#[repr(u32)]
#[derive(Serialize, Deserialize, Debug, PartialOrd, PartialEq, Eq, Copy, Clone)]
pub enum GpuEvent {
    HDraw,
    HBlank,
    VBlankHDraw,
    VBlankHBlank,
}

#[repr(u32)]
#[derive(Serialize, Deserialize, Debug, PartialOrd, PartialEq, Eq, Copy, Clone)]
pub enum ApuEvent {
    Psg1Generate,
    Psg2Generate,
    Psg3Generate,
    Psg4Generate,
    Sample,
}

#[repr(u32)]
#[derive(Serialize, Deserialize, Debug, PartialOrd, PartialEq, Eq, Copy, Clone)]
pub enum EventType {
    RunLimitReached,
    Gpu(GpuEvent),
    Apu(ApuEvent),
    DmaActivateChannel(usize),
    TimerOverflow(usize),
}

#[derive(Serialize, Deserialize, Debug, Clone, Eq)]
pub struct Event {
    typ: EventType,
    /// Timestamp in cycles
    time: usize,
}

impl Event {
    pub fn new(typ: EventType, time: usize) -> Event {
        Event { typ, time }
    }

    #[inline]
    fn get_type(&self) -> EventType {
        self.typ
    }
}

/// Future event is an event to be scheduled in x cycles from now
pub type FutureEvent = (EventType, usize);

impl Ord for Event {
    fn cmp(&self, other: &Self) -> Ordering {
        self.time.cmp(&other.time).reverse()
    }
}

/// Implement custom reverse ordering
impl PartialOrd for Event {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        other.time.partial_cmp(&self.time)
    }

    #[inline]
    fn lt(&self, other: &Self) -> bool {
        other.time < self.time
    }
    #[inline]
    fn le(&self, other: &Self) -> bool {
        other.time <= self.time
    }
    #[inline]
    fn gt(&self, other: &Self) -> bool {
        other.time > self.time
    }
    #[inline]
    fn ge(&self, other: &Self) -> bool {
        other.time >= self.time
    }
}

impl PartialEq for Event {
    fn eq(&self, other: &Self) -> bool {
        self.time == other.time
    }
}

/// Event scheduelr for cycle aware components
/// The scheduler should be "shared" to all event generating components.
/// Each event generator software component can call Scheduler::schedule to generate an event later in the emulation.
/// The scheduler should be updated for each increment in CPU cycles,
///
/// The main emulation loop can then call Scheduler::process_pending to handle the events.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Scheduler {
    timestamp: usize,
    events: BinaryHeap<Event>,
}

impl Scheduler {
    pub fn new() -> Scheduler {
        Scheduler {
            timestamp: 0,
            events: BinaryHeap::with_capacity(NUM_EVENTS),
        }
    }

    #[inline]
    #[allow(unused)]
    pub fn num_pending_events(&self) -> usize {
        self.events.len()
    }

    #[inline]
    #[allow(unused)]
    pub fn peek_next(&self) -> Option<EventType> {
        self.events.peek().map(|e| e.typ)
    }

    /// Schedule an event to be executed in `when` cycles from now
    pub fn schedule(&mut self, event: FutureEvent) {
        let (typ, when) = event;
        let event = Event::new(typ, self.timestamp + when);
        self.events.push(event);
    }

    /// Schedule an event to be executed at an exact timestamp, can be used to schedule "past" events.
    pub fn schedule_at(&mut self, event_typ: EventType, timestamp: usize) {
        self.events.push(Event::new(event_typ, timestamp));
    }

    /// Cancel all events with type `typ`
    /// This method is rather expansive to call since we are reallocating the entire event tree
    pub fn cancel_pending(&mut self, typ: EventType) {
        let mut new_events = BinaryHeap::with_capacity(NUM_EVENTS);
        self.events
            .iter()
            .filter(|e| e.typ != typ)
            .for_each(|e| new_events.push(e.clone()));
        self.events = new_events;
    }

    /// Updates the scheduler timestamp
    #[inline]
    pub fn update(&mut self, cycles: usize) {
        self.timestamp += cycles;
    }

    pub fn pop_pending_event(&mut self) -> Option<(EventType, usize)> {
        if let Some(event) = self.events.peek() {
            if self.timestamp >= event.time {
                // remove the event
                let event = self.events.pop().unwrap_or_else(|| unreachable!());
                Some((event.get_type(), event.time))
            } else {
                None
            }
        } else {
            None
        }
    }

    #[inline]
    pub fn fast_forward_to_next(&mut self) {
        self.timestamp += self.get_cycles_to_next_event();
    }

    #[inline]
    pub fn get_cycles_to_next_event(&self) -> usize {
        if let Some(event) = self.events.peek() {
            event.time - self.timestamp
        } else {
            0
        }
    }

    #[inline]
    /// Safety - Onyl safe to call when we know the event queue is not empty
    pub unsafe fn timestamp_of_next_event_unchecked(&self) -> usize {
        self.events
            .peek()
            .unwrap_or_else(|| std::hint::unreachable_unchecked())
            .time
    }

    #[inline]
    pub fn timestamp(&self) -> usize {
        self.timestamp
    }

    #[allow(unused)]
    fn is_empty(&self) -> bool {
        self.events.is_empty()
    }

    pub fn measure_cycles<F: FnMut()>(&mut self, mut f: F) -> usize {
        let start = self.timestamp;
        f();
        self.timestamp - start
    }
}
