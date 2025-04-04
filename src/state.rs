use crate::channel::State;
use gnss_rs::sv::SV;
use std::collections::HashMap;

pub struct UpdateFunc {
    pub func: Box<dyn Fn() + Send + Sync>,
}

pub struct ChannelState {
    pub state: State,
    pub cn0: f64,
}
impl Default for ChannelState {
    fn default() -> Self {
        Self {
            state: State::Acquisition,
            cn0: 0.0,
        }
    }
}

pub struct GnssState {
    pub channels: HashMap<SV, ChannelState>,
    pub update_func: UpdateFunc,
}

impl GnssState {
    pub fn new() -> Self {
        Self {
            channels: HashMap::<SV, ChannelState>::new(),
            update_func: UpdateFunc {
                func: Box::new(|| {}),
            },
        }
    }
    pub fn set_update_func(&mut self, func: Box<dyn Fn() + Send + Sync>) {
        self.update_func.func = func;
    }
}
