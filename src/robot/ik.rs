use std::f64::{INFINITY, NEG_INFINITY};

use k::{self, UnitQuaternion};
use optimization_engine::constraints::{Rectangle};
use optimization_engine::{constraints::NoConstraints, panoc::*, *};
extern crate nalgebra as na;
use na::{Vector, Vector3};
use crate::robot::state::{State};
use crate::robot::constraints::{Vector7};
use super::constraints::{ParsedConstraints};

use std::time::Duration;


type Arm = k::SerialChain<f64>;


fn finite_difference(f: &dyn Fn(&[f64], &mut f64) -> Result<(), SolverError>, u: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
    let h = 1e-6;
    let mut f0 = 0.0;
    f(u, &mut f0).unwrap();

    let mut x = [0.0,0.0,0.0,0.0,0.0,0.0,0.0];
    for i in 0..7 {
        x[i] = u[i];
    }

    for i in 0..7 {
        let mut fi = 0.0;
        x[i] += h;
        f(&x, &mut fi).unwrap();
        grad[i] = (fi - f0) / h;
        x[i] -= h;
    }

    Ok(())
}


fn position_cost(current_position: &Vector3<f64>, desired_position: &Vector3<f64>) -> f64 {
    (current_position - desired_position).norm_squared()
}

fn rotation_cost(current_rotation: &UnitQuaternion<f64>, desired_rotation: &UnitQuaternion<f64>) -> f64 {
    current_rotation.angle_to(desired_rotation).powf(2.0)
}

fn goal_cost(desired_state: &State, arm: &Arm) -> f64 {
    let mut c = 0.0;
    if let Some(name_transforms) = &desired_state.transforms {
        for named_trans in name_transforms.iter() {
            let trans = arm.find(&named_trans.name).unwrap().world_transform().unwrap();

            if let Some(desired_position) = named_trans.position {
                c += position_cost(&trans.translation.vector, &desired_position);
            }
            
            if let Some(desired_rotation) = named_trans.rotation {
                c += rotation_cost(&trans.rotation, &desired_rotation);
            }
        }
    }
    c
}

fn pow(a: &Vector7, p: f64) -> Vector7 {
    a.map(|_a| _a.powf(p))
}

fn stopping_distance(v0: &Vector7, a0: &Vector7, constraints: &ParsedConstraints, dt: f64) -> Vector7 {
    let q = v0 * dt + a0 * dt.powf(2.) + constraints.joint_jerk_max * dt.powf(3.);
    let v = v0 + a0 * dt + constraints.joint_jerk_max * dt.powf(2.);
    let a = a0 + constraints.joint_jerk_max * dt;

    let jm = -constraints.joint_jerk_max;
    let am = -constraints.joint_accel_max;
    
    let t_a = (am - a).component_div(&jm);
    let v_a = v + a.component_mul(&t_a) + jm.component_mul(&pow(&t_a, 2.)) / 2.;

    let q_a = q + v.component_mul(&t_a) + a.component_mul(&pow(&t_a, 2.)) / 2. + jm.component_mul(&pow(&t_a, 3.)) / 6.;
    let t_v = - v_a.component_div(&am);
    let qt_v = q_a + v_a.component_mul(&t_v) + am.component_mul(&pow(&t_v, 2.)) / 2.;

    let _a = -27. * pow(&(jm / 6.), 2.);
    let _b = 18. * (jm / 6.).component_mul(&(a / 2.)).component_mul(&v)
        - 4. * pow(&(a / 2.), 3.);
    let _c = pow(&(a / 2.), 2.).component_mul(&pow(&v, 2.))
        - 4. * (jm / 6.).component_mul(&pow(&v, 3.));
    
    let desc = pow(&_b, 2.) - 4. * _a.component_mul(&_c);


    let mut res = Vector7::from([0.; 7]);
    for i in 0..7 {
        if v_a[i] > 0. {
            res[i] = qt_v[i];
        } else {
            if desc[i] < 0. {
                res[i] = 0.;
            } else {
                res[i] = (-_b[i] + (_b[i].powf(2.) - 4. * _a[i] * _c[i]).sqrt()) / (2. * _a[i]);
            }
        }
    }
    res
}

fn stopping_v(a0: &Vector7, constraints: &ParsedConstraints, dt: f64) -> Vector7 {
    let v = a0 * dt + constraints.joint_jerk_max * dt.powf(2.);
    let a = a0 + constraints.joint_jerk_max * dt;
    let jm = -constraints.joint_jerk_max;

    v - pow(&a, 2.).component_div(&jm) / 2.
}

fn compute_bounds(state: &State, constraints: &ParsedConstraints, dt: f64) -> ([f64;7], [f64;7]) {
    let q = Vector::from(state.joint_position.map(|p| p.unwrap()));
    let dq = Vector::from(state.joint_velocity.map(|p| p.unwrap()));
    let ddq = Vector::from(state.joint_acceleration.map(|p| p.unwrap()));

    if dq < constraints.joint_vel_min || dq > constraints.joint_vel_max {
        println!("{:?}", dq);
    }

    let dt2 = dt * dt;
    let dt3 = dt2 * dt;

    let zeros = Vector7::from([0.; 7]);
    let buffer = Vector7::from([1e-4;7]);
    let suqb = constraints.joint_max - buffer - stopping_distance(&dq, &ddq, constraints, dt);
    let slqb = constraints.joint_min + buffer + stopping_distance(&-dq, &-ddq, constraints, dt);

    let suvb = constraints.joint_vel_max - buffer - stopping_v(&ddq, constraints, dt);
    let slvb = constraints.joint_vel_min + buffer + stopping_v(&-ddq, constraints, dt);
    
    let suab = ((suvb - dq).sup(&zeros) / dt).inf(&constraints.joint_accel_max);
    let slab = ((slvb - dq).inf(&zeros) / dt).sup(&constraints.joint_accel_min);

    let sujb = ((suab - ddq) / dt).sup(&zeros);
    let sljb = ((slab - ddq) / dt).inf(&zeros);

    let mut sub = suqb
        .inf(&(q + suvb * dt))
        .inf(&(q + dq * dt + suab * dt.powf(2.)))
        .inf(&(q + dq * dt + ddq * dt.powf(2.) + sujb * dt.powf(3.)));
    let mut slb = slqb
        .sup(&(q + slvb * dt))
        .sup(&(q + dq * dt + slab * dt.powf(2.)))
        .sup(&(q + dq * dt + ddq * dt.powf(2.) + sljb * dt.powf(3.)));

    let ujb = (&((constraints.joint_max - q - dq * dt - ddq * dt.powf(2.)) / dt.powf(3.)))
        .inf(&(((constraints.joint_vel_max - dq).sup(&zeros) - ddq * dt) / dt.powf(2.)))
        .inf(&(((constraints.joint_accel_max - ddq).sup(&zeros)) / dt))
        .inf(&constraints.joint_jerk_max);
    let ljb = (&((constraints.joint_min - q - dq * dt - ddq * dt.powf(2.)) / dt.powf(3.)))
        .sup(&(((constraints.joint_vel_min - dq).inf(&zeros) - ddq * dt) / dt.powf(2.)))
        .sup(&(((constraints.joint_accel_min - ddq).inf(&zeros)) / dt))
        .sup(&constraints.joint_jerk_min);


    let hub = q + dq * dt + ddq * dt2 + ujb * dt3;
    let hlb = q + dq * dt + ddq * dt2 + ljb * dt3;
    // println!("ujb: {:?}", ujb);
    // println!("q: {:?}", q);
    // println!("v: {:?}", dq);
    // println!("a: {:?}", ddq);
    // println!("hub j: {:?}", (hub - q) / dt3);
    
    sub = sub.sup(&hlb);
    slb = slb.inf(&hub);

    for i in 0..7 {
        if sub[i] < slb[i] {
            let m = (sub[i] + slb[i]) / 2.;
            sub[i] = m;
            slb[i] = m;
        }
    }


    for i in 0..7 {
        if hub[i] < hlb[i] {
            println!("{:?}", q);
            println!("{:?}", constraints.joint_min);
            println!("{:?}", dq);
            println!("{:?}", constraints.joint_vel_max);
            println!("{:?}", ddq);

            println!("q: {:?}", q[i]);
            println!("ub: {:?}", hub[i]);
            println!("lb: {:?}", hlb[i]);

            println!("suvb: {:?}", suvb[i]);
            println!("suab: {:?}", suab[i]);

            println!("dq: {:?}", dq[i]);
            println!("ddq: {:?}", ddq[i]);
            panic!("");
        }
    }

    let mut lower_bound = [NEG_INFINITY; 7];
    let mut upper_bound = [INFINITY; 7];
    for i in 0..7 {
        lower_bound[i] = constraints.joint_min[i];//hlb[i].max(slb[i]);
        upper_bound[i] = constraints.joint_max[i];//hub[i].min(sub[i]);
    }


    (lower_bound, upper_bound)
}

pub struct Controller {
    panoc_cache : PANOCCache,
    arm : Arm,
    constriants : Option<ParsedConstraints>
}

impl Controller {
    pub fn new(arm: Arm, constraints : Option<ParsedConstraints>) -> Self {
        Controller {
            panoc_cache : panoc::PANOCCache::new(7, 1e-6, 100),
            arm: arm,
            constriants: constraints
        }
    }

    pub fn update(&mut self, state: &State, desired_state: &State, dt: f64) -> [f64; 7] {


        let arm = &mut self.arm;

        let cost = |u: &[f64], c: &mut f64| {
            if let Err(err) = arm.set_joint_positions(u) {
                println!("{}|{:?}", err, u);
                panic!();
            }
            arm.update_transforms();
            *c = goal_cost(desired_state, &arm);
            Ok(())
        };

        let dcost = |u: &[f64], grad: &mut [f64]| {
            finite_difference(&cost, u, grad)
        };

        
        let mut u = [0.0; 7];
        for (ui, qi) in u.iter_mut().zip(state.joint_position) {
            *ui = qi.unwrap();
        }
        
        if let Some(constraints) = &self.constriants {

            //let (lb, ub) = compute_bounds(state, constraints, dt);
            let mut lb = [0.0; 7];
            let mut ub = [0.0; 7];
            for i in 0..7 {
                lb[i] = constraints.joint_min[i];
                ub[i] = constraints.joint_max[i];
            }
            let bounds = Rectangle::new(Some(&lb), Some(&ub));
        

            let problem = Problem::new(
                &bounds,
                 dcost, 
                cost
            );
            let mut panoc = PANOCOptimizer::new(problem, &mut self.panoc_cache)
                        .with_tolerance(1e-6)
                        .with_max_duration(Duration::from_millis(1));
            panoc.solve(&mut u).unwrap();
            //println!("arm: {:?}", arm.find(&"panda_joint8".to_string()).unwrap().world_transform().unwrap());

        } else {
            let bounds = NoConstraints::new();
            let problem = Problem::new(
                &bounds,
                 dcost, 
                cost
            );
            let mut panoc = PANOCOptimizer::new(problem, &mut self.panoc_cache)
                                .with_max_iter(1);
            panoc.solve(&mut u).unwrap();
        }
        //panic!();
        u
    }
}