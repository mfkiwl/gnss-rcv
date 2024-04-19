use std::fmt;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Serialize, Deserialize, PartialEq, Default)]
pub struct IQPair {
    pub i: i8,
    pub q: i8,
}
impl IQPair {
    pub fn new(i: i8, q: i8) -> Self {
        Self { i, q }
    }
}

impl fmt::Debug for IQPair {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "IQPair: {{ i={:4} q={:4} }}",
            self.i, self.q
        )
    }
}