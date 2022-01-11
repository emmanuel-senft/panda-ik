extern crate nalgebra as na;
use na::{Vector3, UnitQuaternion, Quaternion};

use optimization_engine::constraints::{Constraint, Rectangle};
use optimization_engine::{Optimizer, Problem, SolverError, panoc::*};

use std::borrow::BorrowMut;
use std::panic::catch_unwind;
use std::sync::{Mutex};

extern crate libc;
use libc::c_char;
use std::ffi::CStr;
use std::str;

use k::*;

type Robot = k::Chain<f64>;

struct SolverState {
    panoc_cache : PANOCCache,
    robot : Robot
}

static mut STATE: Option<Mutex<SolverState>> = None;

fn charp_to_str<'a>(p : *mut c_char) -> &'a str {
    let c_str : &CStr = unsafe {CStr::from_ptr(p)};
    c_str.to_str().unwrap()
}

fn _init(urdf: &str) {
    let robot = k::Chain::<f64>::from_urdf_file(urdf).unwrap();
    // Create a set of joints from end joint
    let panoc_cache = PANOCCache::new(11, 1e-6, 100000);


    unsafe {
        *STATE.borrow_mut() = Some(
            Mutex::new(
                SolverState {panoc_cache, robot}
            )
        );
    }
}

#[no_mangle]
pub extern "C" fn init(urdf_p: *mut c_char) -> bool {
    
    let urdf = charp_to_str(urdf_p);
    match catch_unwind(|| {
        _init(urdf);
    }) {
        Err(err) => {
            println!("{:?}", err);
            false
        },
        _ => {
            return true;
        }
    }
}

fn get_state<'a>() -> &'a mut Mutex<SolverState> {
    unsafe { STATE.as_mut().unwrap() }
}


pub fn groove_loss(x_val: f64, offset: f64, top_width: f64, bottom_width: i32, depth: f64, poly_factor: f64, g: i32) -> f64 {
    2.*depth*depth-( (-((x_val - offset)/top_width).powi(2*bottom_width)) * (2.0 * depth.powi(2) ) ).exp() + poly_factor * (x_val - offset).powi(g)
}

fn position_cost(current_position: &Vector3<f64>, desired_position: &Vector3<f64>) -> f64 {
    let n = (current_position - desired_position).norm();
    n * n
}

fn movement_cost(state: &[f64], init_state: &[f64],lb: &[f64], hb: &[f64]) -> f64 {
    let mut n = 0.0;
    for i in 0..7 {
        n+=((state[i]-init_state[i])/(hb[i]-lb[i])).powi(2);
    }
    n
}

fn drone_orientation_cost(state: &[f64], destination: &Vector3<f64>) -> f64 {
    let theta = state[10];
    let angle2 = (state[8]-destination[1]).atan2(state[7]-destination[0]);
    let n=(theta.cos()*(state[8]-destination[1])-theta.sin()*(state[7]-destination[0])).powi(2)+((theta-angle2-std::f64::consts::PI)/(2.*std::f64::consts::PI)).powi(2);
    ((state[10]-angle2-std::f64::consts::PI)/(2.*std::f64::consts::PI)).powi(2)
}

fn drone_distance_cost(state: &[f64], destination: &Vector3<f64>, velocity: &Vector3<f64>) -> f64 {
    let n = ((state[7]-destination[0]).powi(2)+(state[8]-destination[1]).powi(2)-(0.5+velocity.norm()/4.).powi(2)).powi(2);
    n
}

fn drone_altitude_cost(state: &[f64], destination: &Vector3<f64>) -> f64 {
    let n=(state[9]-destination[2]).powi(2);
    n
}

fn drone_angle_cost(state: &[f64], destination: &Vector3<f64>, rotation: &UnitQuaternion<f64>) -> f64 {
    let theta=(state[8]-destination[1]).atan2(state[7]-destination[0]);
    (theta-rotation.euler_angles().2).powi(2)
}
    
fn joint_limit_cost(state: &[f64], lb: &[f64], hb: &[f64]) -> f64 {
    let mut n = 0.0;
    for i in 0..7 {
        n+=0.5+groove_loss((state[i]-lb[i]).max(hb[i]-state[i])/(hb[i]-lb[i]), 0.,1.,12,2.,2.,2);
    }
    n
}

fn rotation_cost(current_rotation: &UnitQuaternion<f64>, desired_rotation: &UnitQuaternion<f64>) -> f64 {
    let a = current_rotation.angle_to(desired_rotation);
    a*a
}

fn finite_difference(f: &dyn Fn(&[f64], &mut f64) -> Result<(), SolverError>, u: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
    let h = 1000.0 * f64::EPSILON;
    let mut f0 = 0.0;
    f(u, &mut f0).unwrap();

    let mut x = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
    for i in 0..11 {
        x[i] = u[i];
    }

    for i in 0..11 {
        let mut fi = 0.0;
        x[i] += h;
        f(&x, &mut fi).unwrap();
        grad[i] = (fi - f0) / h;
        x[i] -= h;
    }

    Ok(())
}

#[no_mangle]
pub extern "C" fn solve(q: *mut [f64;11], link_name: *mut c_char, 
                        goal_x: *mut [f64;3], goal_q: *mut [f64;4], vel: *mut [f64;3]) {

    let state = get_state().get_mut().unwrap();
    let name = charp_to_str(link_name);

    let position = unsafe{Vector3::from(std::ptr::read(goal_x))};
    let orientation = unsafe{UnitQuaternion::from_quaternion(Quaternion::from(std::ptr::read(goal_q)))};

    let velocity = unsafe{Vector3::from(std::ptr::read(vel))};

    let robot = &mut state.robot;
    let panoc_cache = &mut state.panoc_cache;

    match robot.find(&name) {
        None => {

            for node in robot.iter() {
                println!("{:?}", (*node.joint()).name);
            }
            println!("Couldn't find: {}", name);
        },
        Some(v) => {

        }
    };

    let mut lb = [
        //Panda Joint
        -2.8973,
        -1.7628,
        -2.8973,
        -3.0718,
        -2.8973,
        -0.0175,
        -2.8973,
        //Drone x,y,z,theta
        -2.,
        -2.,
        0.2,
        -2.*std::f64::consts::PI
    ];
    let mut ub = [
        2.8973,
        1.7628,
        2.8973,
        -0.0698,
        2.8973,
        3.7525,
        2.8973,
        2.,
        2.,
        2.,
        2.*std::f64::consts::PI
    ];

    let mut middle = [0.;11];
    for i in 0..11 {
        middle[i] = (ub[i] + lb[i]) / 2.;
    }

    for i in 0..7 {
        lb[i] += 1e-1;
        ub[i] -= 1e-1;
    }
    let bounds = Rectangle::new(Some(&lb), Some(&ub));

    let mut q_state = unsafe{std::ptr::read(q).clone()};
    let mut init_state = unsafe{std::ptr::read(q).clone()};

    let cost = |u: &[f64], c: &mut f64| {
        robot.set_joint_positions_clamped(&u[0..7]);
        robot.update_transforms();
        let trans = robot.find(&name).unwrap().world_transform().unwrap();
        let destination = position+velocity*0.25;

        *c = 100.0 * position_cost(&trans.translation.vector, &position);
        *c += 10.*rotation_cost(&trans.rotation, &orientation);
        *c += 10.0 * movement_cost(&u, &init_state, &lb, &ub);
        *c += 2.*drone_orientation_cost(&u,&destination);
        *c += 10.*drone_distance_cost(&u,&destination,&velocity);
        *c += drone_altitude_cost(&u,&destination);
        *c += drone_angle_cost(&u,&destination, &orientation);
        *c += 0.1 * joint_limit_cost(&u, &lb, &ub);
        Ok(())
    };

    let dcost = |u: &[f64], grad: &mut [f64]| {
        finite_difference(&cost, u, grad)
    };

    let mut cur_cost: f64 = 0.0;
    cost(&q_state, &mut cur_cost).unwrap();
    let problem = Problem::new(&bounds, dcost, cost);
    let mut opt = PANOCOptimizer::new(problem, panoc_cache).with_max_iter(100);

    bounds.project(&mut q_state);
    let _status = opt.solve(&mut q_state);
    robot.set_joint_positions_clamped(&q_state[0..7]);
    robot.update_transforms();
    let trans = robot.find(&name).unwrap().world_transform().unwrap();

    //println!("{:?}", robot.find(&name).unwrap().world_velocity().unwrap());
    unsafe {
        *q = q_state;
        *goal_x = [trans.translation.x,trans.translation.y,trans.translation.z];
    }
}