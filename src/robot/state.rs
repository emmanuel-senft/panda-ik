extern crate nalgebra as na;
use k::UnitQuaternion;
use na::{Vector3};

#[derive(Clone, Debug)]
pub struct NamedTransform {
    pub name : String,
    pub position : Option<Vector3<f64>>,
    pub rotation : Option<UnitQuaternion<f64>>
}


#[derive(Clone, Debug)]
pub struct State {
    pub joint_position : [Option<f64>; 7],
    pub joint_velocity : [Option<f64>; 7],
    pub joint_acceleration : [Option<f64>; 7],
    pub transforms : Option<Vec<NamedTransform>>
}

impl State {
    pub fn new() -> Self {
        State { 
            joint_position: [None; 7], 
            joint_velocity: [None; 7], 
            joint_acceleration: [None; 7], 
            transforms: None 
        }
    }
}

impl IntoIterator for State {
    type Item = (Option<f64>, Option<f64>, Option<f64>);
    type IntoIter = StateIntoIterator;

    fn into_iter(self) -> Self::IntoIter {
        StateIntoIterator {
            state: self,
            index: 0,
        }
    }
}

pub struct StateIntoIterator {
    state: State,
    index: usize,
}

impl Iterator for StateIntoIterator {
    type Item = (Option<f64>, Option<f64>, Option<f64>);
    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= 7 {
            return None
        }
        self.index += 1;
        return Some((
            self.state.joint_position[self.index-1],
            self.state.joint_velocity[self.index-1],
            self.state.joint_acceleration[self.index-1],
        ))
    }
}
