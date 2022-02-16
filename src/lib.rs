extern crate nalgebra as na;
use na::{Vector3, UnitQuaternion, Quaternion, Rotation3, Unit, Translation3};
use std::convert::TryFrom;

use ncollide3d::math::Point as OtherPoint;
use ncollide3d::nalgebra::Translation3 as OtherTranslation;
use ncollide3d::nalgebra::UnitQuaternion as OtherUnitQuaternion;
use ncollide3d::nalgebra::Quaternion as OtherQuaternion;
use ncollide3d::math::Vector as OtherVector;
use ncollide3d::shape::{Polyline,Segment,Ball,Cuboid,Capsule};
use ncollide3d::query;
use ncollide3d::query::ClosestPoints;
use ncollide3d::nalgebra::geometry::Isometry3;

use optimization_engine::constraints::{Constraint, Rectangle, Ball2};
use optimization_engine::{Optimizer, Problem, SolverError, panoc::*,core::ExitStatus};

use std::borrow::BorrowMut;
use std::panic::catch_unwind;
use std::sync::{Mutex};
use std::cmp;

extern crate libc;
use libc::c_char;
use std::ffi::CStr;
use std::str;

use rand::Rng;

use k::*;

type Robot = k::Chain<f64>;

struct SolverState {
    robot_panoc_cache : PANOCCache,
    drone_panoc_cache : PANOCCache,
    global_drone_panoc_cache : PANOCCache,
    robot : Robot,
    robot2 : Robot
}

struct Plane {
    normal : Vector3<f64>,
    cube : Cuboid<f64>,
    trans : Isometry3<f64>,
    polyline : Polyline<f64>
}

static mut STATE: Option<Mutex<SolverState>> = None;

fn charp_to_str<'a>(p : *mut c_char) -> &'a str {
    let c_str : &CStr = unsafe {CStr::from_ptr(p)};
    c_str.to_str().unwrap()
}

fn _init(urdf: &str) {
    let robot = k::Chain::<f64>::from_urdf_file(urdf).unwrap();
    let robot2 = k::Chain::<f64>::from_urdf_file(urdf).unwrap();
    // Create a set of joints from end joint
    let robot_panoc_cache = PANOCCache::new(7, 1e-6, 100000);
    let drone_panoc_cache = PANOCCache::new(4, 1e-6, 100000);
    let global_drone_panoc_cache = PANOCCache::new(4, 1e-6, 100000);


    unsafe {
        *STATE.borrow_mut() = Some(
            Mutex::new(
                SolverState {robot_panoc_cache, drone_panoc_cache, global_drone_panoc_cache, robot, robot2}
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

fn get_gaussian(x: f64, c: f64, k: i32) -> f64{
    (-(x).powi(k)/(2.*c.powi(k))).exp()
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

fn drone_movement_cost(state: &[f64], init_state: &[f64], margin: f64, k: i32) -> f64 {
    let drone_pose = Vector3::new(state[0],state[1],state[2]);
    let init_pose = Vector3::new(init_state[0],init_state[1],init_state[2]);
    let mut n = 0.0;
    ((drone_pose - init_pose).norm()).powi(k)
}

fn norm_angle(a: f64) -> f64{
    (a+3.*std::f64::consts::PI)%(2.*std::f64::consts::PI)-std::f64::consts::PI
}

fn diff_angle(a1: f64, a2: f64)-> f64 {
    norm_angle(norm_angle(a1)-(a2))
}

fn drone_line_view_cost(state: &[f64], destination: &Vector3<f64>) -> f64 {
    let view = Vector3::new(state[3].cos(), state[3].sin(), 0.);
    return view.angle(&(destination-Vector3::new(state[0],state[1],state[2]))).powi(2);
    //let theta = state[3];
    //let n=(theta.cos()*(state[1]-destination[1])-theta.sin()*(state[0]-destination[0])).powi(2)+((theta-angle2-std::f64::consts::PI)/(2.*std::f64::consts::PI)).powi(2);
    //(theta-std::f64::consts::PI/2.).powi(2)
}

fn drone_distance_cost(state: &[f64], destination: &Vector3<f64>, closest: &Vector3<f64>) -> f64 {
    //let n = ((state[0]-destination[0]).powi(2)+(state[1]-destination[1]).powi(2)-(0.7+velocity.norm()/4.).powi(2)).powi(2);
    let drone_pose = Vector3::new(state[0],state[1],state[2]);
    let n = ((drone_pose-destination).norm()-2.*(destination-closest).norm() - 0.3).powi(2);
    n
}

fn drone_altitude_cost(state: &[f64], destination: &Vector3<f64>) -> f64 {
    let n=(state[2]-destination[2]).powi(2);
    n
    //let n = (state[7]-destination[0]-distance*angle.cos()).powi(2) + (state[8]-destination[1]-distance*angle.sin()).powi(2) + (state[9]-destination[2]-theta.sin()*distance).powi(2) + (state[10]-std::f64::consts::PI-angle).powi(2); // (state[10] - state[7].atan2(state[8])).powi(2)
}

fn drone_polar_cost(state: &[f64], destination: &Vector3<f64>, rotation: &UnitQuaternion<f64>, closest_point: &Vector3<f64>) -> f64 {
    let drone_pose = Vector3::new(state[0],state[1],destination[2]);
    let angle = (destination-closest_point).angle(&(destination-drone_pose));
    return (angle-std::f64::consts::PI*3./4.).powi(2);
}

fn drone_safety(drone_caps: &Cuboid<f64>, drone_iso: &Isometry3<f64>, robot_coll: &Polyline<f64>, robot_size: f64) -> f64 {
    let x = query::distance(drone_iso, drone_caps, &Isometry3::identity(),robot_coll);
    // println!("{}",x);
    get_gaussian(x,robot_size,6)
}

fn drone_robot_occlusion(state: &[f64], destination: &Vector3<f64>,robot_occ: &Polyline<f64>,uncertainty: Vector3<f64>, drone_size: Vector3<f64>, robot_size: f64) -> f64 {
    let view = Segment::new(OtherPoint::new(state[0],state[1],state[2]),OtherPoint::new(destination[0],destination[1],destination[2]));
    let x = query::distance(&Isometry3::identity(), &view, &Isometry3::identity(),robot_occ);
    //println!("{}",x);
    (-(x/(drone_size[0]+robot_size+uncertainty[0])).powi(6)).exp()
}
    


fn joint_limit_cost(state: &[f64], lb: &[f64], hb: &[f64]) -> f64 {
    let mut n = 0.0;
    for i in 0..7 {
        n+=groove_loss((state[i]-lb[i])/(hb[i]-lb[i]), 0.5,0.5,12,2.,2.,2);
    }
    n
}

fn rotation_cost(current_rotation: &UnitQuaternion<f64>, desired_rotation: &UnitQuaternion<f64>) -> f64 {
    let a = current_rotation.angle_to(desired_rotation);
    a*a
}

fn drone_plane_collision_cost(drone_caps: &Cuboid<f64>, drone_iso: &Isometry3<f64>, planes: &Vec<Plane>)->f64{
    let mut safety_cost = 0.;
    for i in 0..planes.len(){
        let safety_dist = query::distance(&planes[i].trans, &planes[i].cube, drone_iso, drone_caps);
        safety_cost += get_gaussian(safety_dist,0.1,6);//(-(safety_dist/0.1).powi(6)).exp();
    }
    safety_cost
}

fn drone_plane_occlusion_cost(state: &[f64], destination: &Vector3<f64>, planes: &Vec<Plane>, uncertainty: Vector3<f64>)->f64{
    let mut occlusion_cost = 0.;
    let segment = Segment::new(OtherPoint::new(state[0],state[1],state[2]),OtherPoint::new(destination[0],destination[1],destination[2]));
    for i in 0..planes.len(){
        let mut occlusion_dist = query::distance(&planes[i].trans, &planes[i].cube, &Isometry3::identity(),&segment);
        if occlusion_dist == 0.{
            occlusion_dist = -query::distance(&Isometry3::identity(), &planes[i].polyline, &Isometry3::identity(),&segment);
        }
        occlusion_cost += (-occlusion_dist+uncertainty[0]).max(0.).powi(2);
    }
    occlusion_cost
}

fn drone_goal_cost(state: &[f64], goal: &[f64],lb: &[f64], ub: &[f64]) -> f64 {
    let mut c = 0.0;
    for i in 0..3{
        c += ((state[i]-goal[i])).powi(2);
    }
    c + (norm_angle(state[3]-goal[3])/std::f64::consts::PI).powi(2)/4.
}

fn robot_finite_difference(f: &dyn Fn(&[f64], &mut f64) -> Result<(), SolverError>, u: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
    let h = 1000.0 * f64::EPSILON;
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

fn drone_finite_difference(f: &dyn Fn(&[f64], &mut f64) -> Result<(), SolverError>, u: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
    let h = 1e-10;
    let mut f0 = 0.0;
    f(u, &mut f0).unwrap();

    let mut x = [0.0,0.0,0.0,0.0];
    for i in 0..4 {
        x[i] = u[i];
    }

    for i in 0..4 {
        let mut fi = 0.0;
        x[i] += h;
        f(&x, &mut fi).unwrap();
        if fi != f0{
            grad[i] = (fi - f0) / h;
        }
        else{
            grad[i] = -f64::EPSILON;
        }
        x[i] -= h;
    }

    Ok(())
}

fn get_point(robot: &Robot, name: &str) -> OtherPoint<f64>{
    let iso = robot.find(name).unwrap().world_transform().unwrap();
    OtherPoint::new(iso.translation.x,iso.translation.y,iso.translation.z)
}

fn get_planes(plane_normals_ptr: *mut f64, plane_points_ptr: *mut f64, plane_centers_ptr: *mut f64,
    plane_orientations_ptr: *mut f64, plane_half_axes_ptr: *mut f64, plane_number_ptr: *mut u8) -> Vec<Plane>{
    let plane_number = unsafe{usize::try_from(std::ptr::read(plane_number_ptr).clone()).unwrap()};
    let normals;
    let points;
    let centers;
    let orientations;
    let half_axes;
    unsafe {
        normals = Vec::from(std::slice::from_raw_parts(plane_normals_ptr, plane_number*3));
        points = Vec::from(std::slice::from_raw_parts(plane_points_ptr, plane_number*12));
        centers = Vec::from(std::slice::from_raw_parts(plane_centers_ptr, plane_number*3));
        orientations = Vec::from(std::slice::from_raw_parts(plane_orientations_ptr, plane_number*4));
        half_axes = Vec::from(std::slice::from_raw_parts(plane_half_axes_ptr, plane_number*3));
    }
    let mut planes = Vec::new();
    for i in 0..plane_number {
        let mut v=Vector3::new(normals[3*i],normals[3*i+1],normals[3*i+2]);
        v=v/v.norm();

        let op1 = OtherPoint::new(points[12*i],points[12*i+1],points[12*i+2]);
        let op2 = OtherPoint::new(points[12*i+3],points[12*i+4],points[12*i+5]);
        let op3 = OtherPoint::new(points[12*i+6],points[12*i+7],points[12*i+8]);
        let op4 = OtherPoint::new(points[12*i+9],points[12*i+10],points[12*i+11]);
        let polyline = Polyline::new(vec![op1,op2,op3,op4],None);

        let center=OtherTranslation::new(centers[3*i],centers[3*i+1],centers[3*i+2]);
        let rotation=OtherUnitQuaternion::from_quaternion(OtherQuaternion::new(orientations[4*i+3],orientations[4*i],orientations[4*i+1],orientations[4*i+2]));
        let half_axes=OtherVector::new(half_axes[3*i],half_axes[3*i+1],half_axes[3*i+2]);

        let cube = Cuboid::new(half_axes);   
        let trans = Isometry3::from_parts(center,rotation);

        let plane = Plane {normal: v, cube : cube, trans : trans, polyline : polyline};

        planes.push(plane);
    }  
    planes
}

#[no_mangle]
pub extern "C" fn solve(robot_start: *mut [f64;7], drone_current: *mut [f64;4], drone_goal: *mut [f64;4], link_name: *mut c_char, 
                        goal_x: *mut [f64;3], goal_q: *mut [f64;4], vel: *mut [f64;3], errors_start: *mut [bool;4],
                        plane_normals_ptr: *mut f64, plane_points_ptr: *mut f64, plane_centers_ptr: *mut f64,
                        plane_orientations_ptr: *mut f64, plane_half_axes_ptr: *mut f64, plane_number_ptr: *mut u8, uncertainty_ptr: *mut [f64;3]) -> f64 {
    
    let planes = get_planes(plane_normals_ptr, plane_points_ptr, plane_centers_ptr, plane_orientations_ptr, plane_half_axes_ptr, plane_number_ptr);
    let position = unsafe{Vector3::from(std::ptr::read(goal_x))};
    let orientation = unsafe{UnitQuaternion::from_quaternion(Quaternion::from(std::ptr::read(goal_q)))};
    let velocity = unsafe{Vector3::from(std::ptr::read(vel))};
    let mut errors = unsafe{std::ptr::read(errors_start).clone()};

    let state = get_state().get_mut().unwrap();
    let robot = &mut state.robot;
    let name = charp_to_str(link_name);

    match robot.find(&name) {
        None => {

            for node in robot.iter() {
                println!("{:?}", (*node.joint()).name);
            }
            println!("Couldn't find: {}", name);
        },
        Some(_v) => {

        }
    };
    let init_state = unsafe{std::ptr::read(robot_start).clone()};
    robot.set_joint_positions_clamped(&init_state);
    robot.update_transforms();

    let (drone_state,cost) = optimize_drone(drone_current, drone_goal, &planes, &position, &orientation, &velocity, &mut errors,robot,uncertainty_ptr);
    let robot_state = optimize_robot(robot_start, &position, &orientation, &mut errors, robot, name, drone_current);
    
    robot.set_joint_positions_clamped(&robot_state);
    robot.update_transforms();
    let trans = robot.find(&name).unwrap().world_transform().unwrap();

    unsafe {
        *robot_start = robot_state;
        *drone_current = drone_state;
        *goal_x = [trans.translation.x,trans.translation.y,trans.translation.z];
        *errors_start = errors;
    }
    cost
}

#[no_mangle]
pub extern "C" fn solveDroneOnly(robot_start: *mut [f64;7], drone_current: *mut [f64;4], drone_goal: *mut [f64;4], last_command: *mut [f64;4], errors_start: *mut [bool;4],
                        plane_normals_ptr: *mut f64, plane_points_ptr: *mut f64, plane_centers_ptr: *mut f64,
                        plane_orientations_ptr: *mut f64, plane_half_axes_ptr: *mut f64, plane_number_ptr: *mut u8, uncertainty_ptr: *mut [f64;3])  {
    
    let planes = get_planes(plane_normals_ptr, plane_points_ptr, plane_centers_ptr, plane_orientations_ptr, plane_half_axes_ptr, plane_number_ptr);
    let mut errors = unsafe{std::ptr::read(errors_start).clone()};

    let state = get_state().get_mut().unwrap();
    let robot = &mut state.robot;
    let name = "panda_gripper_joint";

    match robot.find(&name) {
        None => {

            for node in robot.iter() {
                println!("{:?}", (*node.joint()).name);
            }
            println!("Couldn't find: {}", name);
        },
        Some(_v) => {

        }
    };
    let init_state = unsafe{std::ptr::read(robot_start).clone()};
    robot.set_joint_positions_clamped(&init_state);
    robot.update_transforms();

    let drone_state = optimize_drone_goal(drone_current, drone_goal, last_command, &planes, &mut errors,robot, uncertainty_ptr);

    unsafe {
        *drone_current = drone_state;
        *errors_start = errors;
    }
}

#[no_mangle]
pub extern "C" fn solveGlobalView(robot_start: *mut [f64;7], drone_current: *mut [f64;4], errors_start: *mut [bool;4], plane_normals_ptr: *mut f64, plane_points_ptr: *mut f64, plane_centers_ptr: *mut f64,
                        plane_orientations_ptr: *mut f64, plane_half_axes_ptr: *mut f64, plane_number_ptr: *mut u8, uncertainty_ptr: *mut [f64;3]) -> f64 {
    
    let planes = get_planes(plane_normals_ptr, plane_points_ptr, plane_centers_ptr, plane_orientations_ptr, plane_half_axes_ptr, plane_number_ptr);
    let mut errors = unsafe{std::ptr::read(errors_start).clone()};

    let state = get_state().get_mut().unwrap();
    let robot = &mut state.robot2;
    let name = "panda_gripper_joint";

    match robot.find(&name) {
        None => {

            for node in robot.iter() {
                println!("{:?}", (*node.joint()).name);
            }
            println!("Couldn't find: {}", name);
        },
        Some(_v) => {

        }
    };
    let init_state = unsafe{std::ptr::read(robot_start).clone()};
    robot.set_joint_positions_clamped(&init_state);
    robot.update_transforms();

    let (drone_state,cost) = optimize_global_view(drone_current, &planes, &mut errors,robot, uncertainty_ptr);

    unsafe {
        *drone_current = drone_state;
        *errors_start = errors;
    }
    cost
}

fn optimize_robot(robot_start: *mut [f64;7], position: &Vector3<f64>, orientation: &UnitQuaternion<f64>, errors: &mut [bool;4],robot: &mut Robot, name: &str, drone_c: *mut [f64;4]) -> [f64; 7] {
    let state = get_state().get_mut().unwrap();
    let drone_current = unsafe{std::ptr::read(drone_c).clone()};

    let panoc_cache = &mut state.robot_panoc_cache;

    let mut lb = [
        //Panda Joint
        -2.8973,
        -1.7628,
        -2.8973,
        -3.0718,
        -2.8973,
        -0.0175,
        -2.8973,
    ];

    let mut ub = [
        2.8973,
        1.7628,
        2.8973,
        -0.0698,
        2.8973,
        3.7525,
        2.8973,
    ];

    let mut middle = [0.;7];
    for i in 0..7 {
        middle[i] = (ub[i] + lb[i]) / 2.;
    }

    for i in 0..7 {
        lb[i] += 1e-1;
        ub[i] -= 1e-1;
    }
    let bounds = Rectangle::new(Some(&lb), Some(&ub));

    let mut robot_state = unsafe{std::ptr::read(robot_start).clone()};
    let init_state = unsafe{std::ptr::read(robot_start).clone()};

    let cost = |u: &[f64], c: &mut f64| {
        for i in 0..7{
            if u[i].is_nan(){
                *c = 10.;
                return Ok(())
            }
        }
        robot.set_joint_positions_clamped(&u);
        robot.update_transforms();
        let trans = robot.find(&name).unwrap().world_transform().unwrap();

        *c = 100.0 * position_cost(&trans.translation.vector, &position);
        *c += 10.*rotation_cost(&trans.rotation, &orientation);
        *c += movement_cost(&u, &init_state, &lb, &ub);
        *c += 0.1 * joint_limit_cost(&u, &lb, &ub);
        Ok(())
    };

    let dcost = |u: &[f64], grad: &mut [f64]| {
        robot_finite_difference(&cost, u, grad)
    };

    let mut cur_cost: f64 = 0.0;
    cost(&robot_state, &mut cur_cost).unwrap();
    let problem = Problem::new(&bounds, dcost, cost);
    let mut opt = PANOCOptimizer::new(problem, panoc_cache).with_max_iter(50).with_max_duration(std::time::Duration::from_millis(7));

    bounds.project(&mut robot_state);
    let _status = opt.solve(&mut robot_state);
    if _status.is_err(){
        errors[0]=true;
    }
    else{
        if _status.unwrap().exit_status() == ExitStatus::NotConvergedOutOfTime {
            errors[1]=true;
        }
        else{}
        if _status.unwrap().exit_status() == ExitStatus::NotConvergedIterations {
            errors[1]=true;
        }
        else{}
    }
    robot_state
}

fn get_drone_boundary(last_command: [f64; 4])->([f64; 4],[f64; 4]) {
    let margin = 0.01;
    let ub = [
        (2.).min(last_command[0]+margin),
        (2.).min(last_command[1]+margin),
        (2.).min(last_command[2]+margin),
        (2.*std::f64::consts::PI).min(last_command[3]+10.*margin)
    ];
    let lb = [
        //Drone x,y,z,theta
        (-2.).max(last_command[0]-margin),
        (-2.).max(last_command[1]-margin),
        (0.2).max(last_command[2]-margin),
        (-2.*std::f64::consts::PI).max(last_command[3]-10.*margin)
    ];

    (ub,lb)
}

fn drone_collision_cost(state: &[f64], current_position: &[f64], planes: &Vec<Plane>, robot_coll: &Polyline<f64>, uncertainty: Vector3<f64>, drone_size: Vector3<f64>, robot_size: f64) -> f64{
    let drone_caps = Cuboid::new(OtherVector::new(drone_size[0]+uncertainty[0],drone_size[1]+uncertainty[1],drone_size[2]+uncertainty[2]));
    let trans = OtherTranslation::new(state[0],state[1],state[2]);
    let rot = OtherUnitQuaternion::from_quaternion(OtherQuaternion::new(0.,0.,0.,1.));
    let drone_iso = &Isometry3::from_parts(trans, rot);

    let c1 = 10.*drone_plane_collision_cost(&drone_caps, &drone_iso, &planes); 
    let c2 = 10.*drone_safety(&drone_caps, &drone_iso, robot_coll, robot_size);
    c1+c2
}

fn drone_motion_cost(state: &[f64], current_position: &[f64]) -> f64{
    1000.*drone_movement_cost(&state, &current_position, 0.3, 6)
}


fn drone_view_cost(state: &[f64], destination: &Vector3<f64>, orientation: &UnitQuaternion<f64>, planes: &Vec<Plane>, closest_point: &Vector3<f64>, robot_occ: &Polyline<f64>, uncertainty: Vector3<f64>, drone_size: Vector3<f64>, robot_size: f64)->f64{
    let mut c1 = ((state[0]-destination[0]).powi(2)+(state[1]-destination[1]).powi(2)-(0.7_f64).powi(2)).powi(2);
    let mut c2 = 0.;
    if planes.len() > 0 {
        c1 = 10.*drone_distance_cost(&state,&destination,&closest_point);
        c2 = 5.*drone_polar_cost(&state,&destination, &orientation,&closest_point);
    }
    else{
        let theta=(state[1]-destination[1]).atan2(state[0]-destination[0]);
        c2 = (norm_angle(theta)-orientation.euler_angles().2).powi(2);
    }

    let c3 =  10.*drone_line_view_cost(&state,&destination);
    let c4 = 200. * drone_plane_occlusion_cost(&state,&destination,&planes,uncertainty); 
    let c5 = 20.*drone_robot_occlusion(&state,&destination,&robot_occ,uncertainty,drone_size, robot_size);
    c1+c2+c3+c4+c5

}

fn get_closest_plane(ee_position: &Vector3<f64>, planes:&Vec<Plane> ) -> Vector3<f64>{
    //Closest plane
    let mut closest_point = Vector3::new(-1.,-1.,-1.);
    if planes.len() > 0 {
        let ee_ball = Ball::new(0.01);
        let trans = Isometry3::new(OtherVector::new(ee_position[0], ee_position[1], ee_position[2]), na::zero());
        let mut index = 0;
        let mut min_dist = f64::MAX;

        for i in 0..planes.len(){
            let dist = query::distance(&planes[i].trans, &planes[i].cube, &trans,&ee_ball);
            if dist < min_dist {
                min_dist = dist;
                index = i;
            }
        }
        let plane = &planes[index];
        let cp = query::closest_points(&plane.trans, &plane.cube, &trans, &ee_ball,200.);
        match cp{
                ClosestPoints::Intersecting=>{
            },
                ClosestPoints::WithinMargin(p1,p2)=>{
                    closest_point[0] = p1[0];
                    closest_point[1] = p1[1];
                    closest_point[2] = p1[2];
            },
            ClosestPoints::Disjoint=>{
            }
        };
    }
    closest_point
}

fn optimize_drone(drone_c: *mut [f64;4], drone_goal: *mut [f64;4], planes: &Vec<Plane>, position: &Vector3<f64>, orientation: &UnitQuaternion<f64>, velocity: &Vector3<f64>, errors: &mut [bool;4], robot: &mut Robot, uncertainty_ptr: *mut [f64;3]) -> ([f64; 4],f64) {
    let mut drone_state = unsafe{std::ptr::read(drone_goal).clone()};
    let uncertainty = unsafe{Vector3::try_from(std::ptr::read(uncertainty_ptr).clone()).unwrap()};
    let drone_size = Vector3::new(0.12,0.12,0.025);
    let robot_size = 0.1;
    let drone_current = unsafe{std::ptr::read(drone_c).clone()};
    let last_command = unsafe{std::ptr::read(drone_goal).clone()};
    let _init_state = unsafe{std::ptr::read(drone_c).clone()};

    let (ub,lb) = get_drone_boundary(last_command);
    let bounds = Rectangle::new(Some(&lb), Some(&ub));
    let state = get_state().get_mut().unwrap();
    let panoc_cache = &mut state.drone_panoc_cache;

    let destination = position;//+velocity*0.25;

    let p1 = get_point(robot,"panda_joint1");
    let p2 = get_point(robot,"panda_joint3");
    let p3 = get_point(robot,"panda_joint4");
    let p4 = get_point(robot,"panda_joint6");
    let p5 = get_point(robot,"panda_joint7");
    let p6 = get_point(robot,"panda_gripper_joint");
    let robot_occ = Polyline::new(vec![p1,p2,p3,p4,p5],None);
    let robot_coll = Polyline::new(vec![p1,p2,p3,p4,p5,p6],None);

    let closest_point = get_closest_plane(&destination,planes);

    let cost = |u: &[f64], c: &mut f64| {
        let c_collision = drone_collision_cost(&u,&drone_current,&planes,&robot_coll,uncertainty,drone_size,robot_size);
        let c_motion = drone_motion_cost(&u,&drone_current);
        let c_view = drone_view_cost(&u, &destination, &orientation, &planes, &closest_point, &robot_occ, uncertainty, drone_size, robot_size);
        *c=c_collision+c_motion+c_view;
        
        
        Ok(())
    };

    let dcost = |u: &[f64], grad: &mut [f64]| {
        drone_finite_difference(&cost, u, grad)
    };

    let mut cur_cost = 0.0;
    cost(&drone_state, &mut cur_cost).unwrap();
    let problem = Problem::new(&bounds, dcost, cost);
    let mut opt = PANOCOptimizer::new(problem, panoc_cache).with_max_iter(50).with_max_duration(std::time::Duration::from_millis(10));

    bounds.project(&mut drone_state);
    let _status = opt.solve(&mut drone_state);
    drone_state[3] = norm_angle(drone_state[3]);
    if _status.is_err(){
        errors[2]=true;
    }
    else{
        if _status.unwrap().exit_status() == ExitStatus::NotConvergedOutOfTime {
            errors[3];
        }
        else{}
    }
    let cost = drone_view_cost(&drone_state, &destination, &orientation, &planes, &closest_point, &robot_occ, uncertainty, drone_size, robot_size)
            + drone_collision_cost(&drone_state,&drone_current,&planes,&robot_coll,uncertainty,drone_size,robot_size);
    (drone_state,cost)
}

fn optimize_drone_goal(drone_c: *mut [f64;4], drone_goal: *mut [f64;4], last_command: *mut [f64;4], planes: &Vec<Plane>, errors: &mut [bool;4], robot: &mut Robot, uncertainty_ptr: *mut [f64;3]) -> [f64; 4] {
    let mut drone_state = unsafe{std::ptr::read(drone_c).clone()};
    let uncertainty = unsafe{Vector3::try_from(std::ptr::read(uncertainty_ptr).clone()).unwrap()};
    
    let drone_size = Vector3::new(0.12,0.12,0.025);
    let robot_size = 0.1;
    let drone_current = unsafe{std::ptr::read(drone_c).clone()};
    let mut drone_goal = unsafe{std::ptr::read(drone_goal).clone()};
    let last_command = unsafe{std::ptr::read(last_command).clone()};

    let (ub,lb) = get_drone_boundary(last_command);
    let bounds = Rectangle::new(Some(&lb), Some(&ub));

    let state = get_state().get_mut().unwrap();
    let panoc_cache = &mut state.drone_panoc_cache;

    drone_goal[3] = norm_angle(drone_goal[3]);
    let p1 = get_point(robot,"panda_joint1");
    let p2 = get_point(robot,"panda_joint3");
    let p3 = get_point(robot,"panda_joint4");
    let p4 = get_point(robot,"panda_joint6");
    let p5 = get_point(robot,"panda_joint7");
    let p6 = get_point(robot,"panda_gripper_joint");
    let robot_coll = Polyline::new(vec![p1,p2,p3,p4,p5,p6],None);
    
    let cost = |u: &[f64], c: &mut f64| {
        let c_collision = drone_collision_cost(&u,&drone_current,&planes,&robot_coll,uncertainty,drone_size,robot_size);
        let c_motion = drone_motion_cost(&u,&drone_current);
        let c_goal = 2.*drone_goal_cost(&u, &drone_goal, &lb, &ub);
        *c=c_collision+c_motion+c_goal;
        Ok(())
    };

    let dcost = |u: &[f64], grad: &mut [f64]| {
        drone_finite_difference(&cost, u, grad)
    };

    let mut cur_cost = 0.0;
    cost(&last_command, &mut cur_cost).unwrap();
    let problem = Problem::new(&bounds, dcost, cost);
    let mut opt = PANOCOptimizer::new(problem, panoc_cache).with_max_iter(50).with_max_duration(std::time::Duration::from_millis(10));

    bounds.project(&mut drone_state);
    let _status = opt.solve(&mut drone_state);
    if _status.is_err(){
        errors[2]=true;
        
    }
    else{
        if _status.unwrap().exit_status() == ExitStatus::NotConvergedIterations {
            errors[3]=true;
        }
        else{}
    }
    drone_state
}

fn optimize_global_view(drone_c: *mut [f64;4], planes: &Vec<Plane>, errors: &mut [bool;4], robot: &mut Robot, uncertainty_ptr: *mut [f64;3]) -> ([f64; 4],f64) {
    let mut drone_state = unsafe{std::ptr::read(drone_c).clone()};
    let uncertainty = unsafe{Vector3::try_from(std::ptr::read(uncertainty_ptr).clone()).unwrap()};
    
    
    let drone_size = Vector3::new(0.12,0.12,0.025);
    let robot_size = 0.1;
    let drone_current = unsafe{std::ptr::read(drone_c).clone()};

    let ub = [
        2.,
        2.,
        2.,
        2.*std::f64::consts::PI
    ];
    let lb = [
        //Drone x,y,z,theta
        -2.,
        -2.,
        0.2,
        -2.*std::f64::consts::PI
    ];

    let bounds = Rectangle::new(Some(&lb), Some(&ub));

    let state = get_state().get_mut().unwrap();
    let panoc_cache = &mut state.global_drone_panoc_cache;

    let p1 = get_point(robot,"panda_joint1");
    let p2 = get_point(robot,"panda_joint3");
    let p3 = get_point(robot,"panda_joint4");
    let p4 = get_point(robot,"panda_joint6");
    let p5 = get_point(robot,"panda_joint7");
    let p6 = get_point(robot,"panda_gripper_joint");
    let robot_occ = Polyline::new(vec![p1,p2,p3,p4,p5],None);
    let robot_coll = Polyline::new(vec![p1,p2,p3,p4,p5,p6],None);

    let iso = robot.find("panda_gripper_joint").unwrap().world_transform().unwrap();
    let destination = iso.translation.vector;
    let orientation = iso.rotation;

    let closest_point = get_closest_plane(&destination,planes);
    
    let cost = |u: &[f64], c: &mut f64| {
        let c_view = drone_view_cost(&u, &destination, &orientation, &planes, &closest_point, &robot_occ, uncertainty, drone_size, robot_size);
        let c_collision = drone_collision_cost(&u,&drone_current,&planes,&robot_coll,uncertainty,drone_size,robot_size);
        
        *c=c_view+c_collision;
        Ok(())
    };

    let dcost = |u: &[f64], grad: &mut [f64]| {
        drone_finite_difference(&cost, u, grad)
    };
    let mut rng = rand::thread_rng();
    drone_state[0] = rng.gen_range(lb[0],ub[0]);
    drone_state[1] = rng.gen_range(lb[1],ub[1]);
    drone_state[2] = rng.gen_range(lb[2],ub[2]);
    drone_state[3] = rng.gen_range(lb[3],ub[3]);
    let mut cur_cost = 0.0;
    cost(&drone_state, &mut cur_cost).unwrap();
    let problem = Problem::new(&bounds, dcost, cost);
    let mut opt = PANOCOptimizer::new(problem, panoc_cache).with_max_iter(500).with_max_duration(std::time::Duration::from_millis(100));

    bounds.project(&mut drone_state);
    let _status = opt.solve(&mut drone_state);
    if _status.is_err(){
        errors[2]=true; 
    }
    else{
        if _status.unwrap().exit_status() == ExitStatus::NotConvergedIterations {
            errors[3]=true;
        }
        else{}
    }
    let cost = drone_view_cost(&drone_state, &destination, &orientation, &planes, &closest_point, &robot_occ, uncertainty, drone_size, robot_size)
            + drone_collision_cost(&drone_state,&drone_current,&planes,&robot_coll,uncertainty,drone_size,robot_size);
    (drone_state,cost)
}
