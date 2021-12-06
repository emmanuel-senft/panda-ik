use std::f64::{INFINITY, NEG_INFINITY};

use serde::{Serialize, Deserialize};
extern crate nalgebra as na;
use na::{Vector, Matrix};

pub type Vector7 = Matrix<f64, na::U7, na::U1, na::ArrayStorage<f64, 7_usize, 1_usize>>;

pub struct ParsedConstraints {
    pub joint_min : Vector7,
    pub joint_max : Vector7,
    pub joint_vel_min : Vector7,
    pub joint_vel_max : Vector7,
    pub joint_accel_min : Vector7,
    pub joint_accel_max : Vector7,
    pub joint_jerk_min : Vector7,
    pub joint_jerk_max : Vector7
}

impl Default for ParsedConstraints {
    fn default() -> ParsedConstraints {
        let neg = Vector::from([NEG_INFINITY; 7]);
        let pos = Vector::from([INFINITY; 7]);
        ParsedConstraints {
            joint_min : neg,
            joint_max : pos,
            joint_vel_min : neg,
            joint_vel_max : pos,
            joint_accel_min : neg,
            joint_accel_max : pos,
            joint_jerk_min : neg,
            joint_jerk_max : pos
        }
    }
}

impl ParsedConstraints {
    pub fn from_robot_constraints(rc: RobotConstraints) -> Self {
        let buffer = Vector7::from([1e-3; 7]);

        let mut this = Self{..Default::default()};

        let extract_bounds = |jcs: &[JointConstraint; 7]| {
            let mut lower = Vector::from([0.0; 7]);
            let mut upper = Vector::from([0.0; 7]);
            for i in 0..7 {
                lower[i] = jcs[i].min.unwrap_or(NEG_INFINITY);
                upper[i] = jcs[i].max.unwrap_or(INFINITY);
            }
            (lower, upper)
        };

        if let Some(jv) = rc.joint_value {
            let (lower, upper) = extract_bounds(&jv);
            this.joint_min = lower + buffer;
            this.joint_max = upper - buffer;
        };
        if let Some(jv) = rc.joint_vel {
            let (lower, upper) = extract_bounds(&jv);
            this.joint_vel_min = lower + buffer;
            this.joint_vel_max = upper - buffer;
        };
        if let Some(jv) = rc.joint_accel {
            let (lower, upper) = extract_bounds(&jv);
            this.joint_accel_min = lower + buffer;
            this.joint_accel_max = upper - buffer;
        };
        if let Some(jv) = rc.joint_jerk {
            let (lower, upper) = extract_bounds(&jv);
            this.joint_jerk_min = lower + buffer;
            this.joint_jerk_max = upper - buffer;
        };

        this
    }
}


#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct JointConstraint {
    pub max : Option<f64>,
    pub min : Option<f64>
}

#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct RobotConstraints {
    pub joint_value : Option<[JointConstraint; 7]>,
    pub joint_vel : Option<[JointConstraint; 7]>,
    pub joint_accel : Option<[JointConstraint; 7]>,
    pub joint_jerk : Option<[JointConstraint; 7]>
}
