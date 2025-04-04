use crate::channel::State;
use gnss_rs::sv::SV;
use std::collections::HashMap;

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

#[derive(Default)]
pub struct GnssState {
    pub channels: HashMap<SV, ChannelState>,
}
