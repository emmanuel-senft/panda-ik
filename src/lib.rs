extern crate nalgebra as na;
use na::{Vector3, UnitQuaternion, Quaternion, Rotation3, Unit};
use std::convert::TryFrom;

use ncollide3d::math::Point as Otherpoint;
use ncollide3d::math::Vector as OtherVector;
use ncollide3d::shape::{Polyline,Segment,Ball};
use ncollide3d::query;
use ncollide3d::nalgebra::geometry::Isometry3;

use optimization_engine::constraints::{Constraint, Rectangle};
use optimization_engine::{Optimizer, Problem, SolverError, panoc::*,core::ExitStatus};

use std::borrow::BorrowMut;
use std::panic::catch_unwind;
use std::sync::{Mutex};
use std::cmp;

extern crate libc;
use libc::c_char;
use std::ffi::CStr;
use std::str;

use geo_types::{Point,LineString, Polygon};
use geo::Closest;
use geo::algorithm::euclidean_distance::EuclideanDistance;
use geo::algorithm::closest_point::ClosestPoint;

use k::*;

type Robot = k::Chain<f64>;

struct SolverState {
    robot_panoc_cache : PANOCCache,
    drone_panoc_cache : PANOCCache,
    robot : Robot
}

struct Plane {
    normal : Vector3<f64>,
    points : [Vector3<f64>;4],
    rot : Rotation3<f64>,
    poly : Polygon<f64>
}

static mut STATE: Option<Mutex<SolverState>> = None;

fn charp_to_str<'a>(p : *mut c_char) -> &'a str {
    let c_str : &CStr = unsafe {CStr::from_ptr(p)};
    c_str.to_str().unwrap()
}

fn _init(urdf: &str) {
    let robot = k::Chain::<f64>::from_urdf_file(urdf).unwrap();
    // Create a set of joints from end joint
    let robot_panoc_cache = PANOCCache::new(7, 1e-6, 100000);
    let drone_panoc_cache = PANOCCache::new(4, 1e-6, 100000);


    unsafe {
        *STATE.borrow_mut() = Some(
            Mutex::new(
                SolverState {robot_panoc_cache, drone_panoc_cache, robot}
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

fn drone_movement_cost(state: &[f64], init_state: &[f64],lb: &[f64], hb: &[f64]) -> f64 {
    let mut n = 0.0;
    for i in 0..3 {
        n+=100000.*(state[i]-init_state[i]).powi(6);
    }
    n
}

fn drone_view_cost(state: &[f64], destination: &Vector3<f64>) -> f64 {
    //let theta = state[3];
    let angle2 = (state[1]-destination[1]).atan2(state[0]-destination[0]);
    //let n=(theta.cos()*(state[1]-destination[1])-theta.sin()*(state[0]-destination[0])).powi(2)+((theta-angle2-std::f64::consts::PI)/(2.*std::f64::consts::PI)).powi(2);
    //(theta-std::f64::consts::PI/2.).powi(2)
    ((state[3]-angle2-std::f64::consts::PI)/(2.*std::f64::consts::PI)).powi(2)
}

fn drone_distance_cost(state: &[f64], destination: &Vector3<f64>, velocity: &Vector3<f64>) -> f64 {
    let n = ((state[0]-destination[0]).powi(2)+(state[1]-destination[1]).powi(2)-(0.7+velocity.norm()/4.).powi(2)).powi(2);
    n
}

fn drone_altitude_cost(state: &[f64], destination: &Vector3<f64>) -> f64 {
    let n=(state[2]-destination[2]).powi(2);
    n
    //let n = (state[7]-destination[0]-distance*angle.cos()).powi(2) + (state[8]-destination[1]-distance*angle.sin()).powi(2) + (state[9]-destination[2]-theta.sin()*distance).powi(2) + (state[10]-std::f64::consts::PI-angle).powi(2); // (state[10] - state[7].atan2(state[8])).powi(2)
}

fn drone_angle_cost(state: &[f64], destination: &Vector3<f64>, rotation: &UnitQuaternion<f64>) -> f64 {
    //let angle1 = destination[1].atan2(destination[0]);
    //let angle2 = state[8].atan2(state[7]);
    //let n = (angle1-robot_rot_mat[1,0].atan2(robot_rot_mat[0,0])).powi(2);
    let theta=(state[1]-destination[1]).atan2(state[0]-destination[0]);
    (theta-rotation.euler_angles().2).powi(2)
}

fn drone_safety(state: &[f64], destination: &Vector3<f64>,robot_coll: &Polyline<f64>) -> f64 {
    let drone_ball = Ball::new(0.10);
    let x = query::distance(&Isometry3::new(OtherVector::new(state[0], state[1], state[2]), na::zero()), &drone_ball, &Isometry3::identity(),robot_coll);
    (-(x/0.2).powi(6)).exp()
}

fn drone_robot_occlusion(state: &[f64], destination: &Vector3<f64>,robot_occ: &Polyline<f64>) -> f64 {
    let view = Segment::new(Otherpoint::new(state[0],state[1],state[2]),Otherpoint::new(destination[0],destination[1],destination[2]));
    let x = query::distance(&Isometry3::identity(), &view, &Isometry3::identity(),robot_occ);
    println!("{}",x);
    (-(x/0.2).powi(6)).exp()
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

fn drone_plane_cost(state: &[f64], destination: &Vector3<f64>, planes: &Vec<Plane>)->(f64,f64){
    let drone = Vector3::new(state[0],state[1],state[2]);

    let mut occlusion_cost = 0.;
    let mut safety_cost = 0.;
    for i in 0..planes.len(){
        let (occlusion_dist,safety_dist) = get_collision(&planes[i],&drone,&destination);
        occlusion_cost += (-occlusion_dist+0.1).max(0.).powi(2);
        safety_cost += 1./(safety_dist.powi(8))*(0.15_f64).powi(8)/25.;
    }
    (occlusion_cost,safety_cost)
}

fn get_2D(R: &Rotation3<f64>, p: &Vector3<f64>)->(f64,f64){
    let v = R*p;
    (v[0],v[1])
}

fn get_3D(R: &Rotation3<f64>, p: &(f64,f64),normal:&Vector3<f64>, plane_point:&Vector3<f64>)->Vector3<f64>{
    line_plane_collision(normal, plane_point, normal, &(R.inverse() * Vector3::new(p.0,p.1,0.)))
}

fn get_norm_rot(v:&Vector3<f64>) -> Rotation3<f64>{
    let ref_vec = Vector3::new(0.,0.,1.);
    let norm_v = Unit::new_normalize(v.cross(&ref_vec));
    Rotation3::from_axis_angle(&norm_v, ref_vec.angle(&v))
}

fn line_plane_collision(plane_normal:&Vector3<f64>, plane_point:& Vector3<f64>, ray_direction:&Vector3<f64>, ray_point:&Vector3<f64>) -> Vector3<f64>{
	let ndotu = plane_normal.dot(ray_direction);
	//if abs(ndotu) < epsilon:
    //    println!("error");

	let w = ray_point - plane_point;
	let si = -plane_normal.dot(&w) / ndotu;
	w + si * ray_direction + plane_point
}

fn get_collision(plane:&Plane, d:&Vector3<f64>, ee:&Vector3<f64>) -> (f64,f64){
    let normal = plane.normal;
    let points = plane.points;
    let psi = line_plane_collision(&normal,&points[0],&(d-ee),&d);

    let inter = get_2D(&plane.rot,&psi);
    let inter_2d = Point::new(inter.0,inter.1);
    let dist = inter_2d.euclidean_distance(&plane.poly);
    let closest = plane.poly.closest_point(&inter_2d);

    let mut closest_point_2d = Point::new(-1.,-1.);
    match closest{
        Closest::Intersection(p)=>{

        },
        Closest::SinglePoint(p)=>{
            closest_point_2d = p;
        },
        Closest::Indeterminate=>{
        }
    };

    let mut safety_dist = (d - psi).norm();
    let mut occlusion_dist = inter_2d.euclidean_distance(&closest_point_2d);

    if  dist==0. {
        occlusion_dist = -occlusion_dist;
    }
    else{
        safety_dist = (d - get_3D(&plane.rot, &(closest_point_2d.x(),closest_point_2d.y()), &normal,&points[0])).norm()
    }
    //wall behind drone
    if (psi-ee).dot(&(psi-d)) > 0.{
        occlusion_dist = safety_dist;
    }

    (occlusion_dist,safety_dist)
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
    let h = 1000.0 * f64::EPSILON;
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
        grad[i] = (fi - f0) / h;
        x[i] -= h;
    }

    Ok(())
}

fn get_point(robot: &Robot, name: &str) -> Otherpoint<f64>{
    let iso = robot.find(name).unwrap().world_transform().unwrap();
    Otherpoint::new(iso.translation.x,iso.translation.y,iso.translation.z)
}

#[no_mangle]
pub extern "C" fn solve(robot_start: *mut [f64;7], drone_current: *mut [f64;4], drone_goal: *mut [f64;4], link_name: *mut c_char, 
                        goal_x: *mut [f64;3], goal_q: *mut [f64;4], vel: *mut [f64;3], errors_start: *mut [bool;4],
                        plane_normals_ptr: *mut f64, plane_points_ptr: *mut f64, plane_number_ptr: *mut u8) {
    
    let plane_number = unsafe{usize::try_from(std::ptr::read(plane_number_ptr).clone()).unwrap()};
    let normals;
    let points;
    unsafe {
        normals = Vec::from(std::slice::from_raw_parts(plane_normals_ptr, plane_number*3));
        points = Vec::from(std::slice::from_raw_parts(plane_points_ptr, plane_number*12));
    }
    let mut planes = Vec::new();
    for i in 0..plane_number {
        let p1 = Vector3::new(points[12*i],points[12*i+1],points[12*i+2]);
        let p2 = Vector3::new(points[12*i+3],points[12*i+4],points[12*i+5]);
        let p3 = Vector3::new(points[12*i+6],points[12*i+7],points[12*i+8]);
        let p4 = Vector3::new(points[12*i+9],points[12*i+10],points[12*i+11]);
        let points = [p1,p2,p3,p4];
        let mut v=Vector3::new(normals[3*i],normals[3*i+1],normals[3*i+2]);
        v=v/v.norm();

        let r = get_norm_rot(&v);
        let poly = Polygon::new(
            LineString::from(vec![get_2D(&r,&p1), get_2D(&r,&p2), get_2D(&r,&p3), get_2D(&r,&p4)]),
            vec![],
        );

        let plane = Plane {normal: v,points: points, rot : r, poly : poly};

        planes.push(plane);
    }                       
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

    let drone_state = optimize_drone(drone_current, drone_goal, &planes, &position, &orientation, &velocity, &mut errors,robot);
    let robot_state = optimize_robot(robot_start, &position, &orientation, &mut errors, robot, name);
    robot.set_joint_positions_clamped(&robot_state);
    robot.update_transforms();
    let trans = robot.find(&name).unwrap().world_transform().unwrap();

    unsafe {
        *robot_start = robot_state;
        *drone_goal = drone_state;
        *goal_x = [trans.translation.x,trans.translation.y,trans.translation.z];
        *errors_start = errors;
    }
}
fn optimize_robot(robot_start: *mut [f64;7], position: &Vector3<f64>, orientation: &UnitQuaternion<f64>, errors: &mut [bool;4],robot: &mut Robot, name: &str) -> [f64; 7] {
    let state = get_state().get_mut().unwrap();

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
    let mut opt = PANOCOptimizer::new(problem, panoc_cache).with_max_iter(100);//.with_max_duration(std::time::Duration::from_millis(7));

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


fn optimize_drone(drone_c: *mut [f64;4], drone_goal: *mut [f64;4], planes: &Vec<Plane>, position: &Vector3<f64>, orientation: &UnitQuaternion<f64>, velocity: &Vector3<f64>, errors: &mut [bool;4], robot: &mut Robot) -> [f64; 4] {
    let mut drone_state = unsafe{std::ptr::read(drone_goal).clone()};
    let drone_current = unsafe{std::ptr::read(drone_c).clone()};
    let _init_state = unsafe{std::ptr::read(drone_goal).clone()};

    let ub = [
        2.,
        2.,
        2.,
        2.*std::f64::consts::PI
    ];
    let lb = [
        //Drone x,y,z,theta
        -2.,
        -0.6,
        0.2,
        -2.*std::f64::consts::PI
    ];
    let mut middle = [0.;4];
    for i in 0..4 {
        middle[i] = (ub[i] + lb[i]) / 2.;
    }
    let bounds = Rectangle::new(Some(&lb), Some(&ub));

    let state = get_state().get_mut().unwrap();
    let panoc_cache = &mut state.drone_panoc_cache;

    let destination = position+velocity*0.25;

    let p1 = get_point(robot,"panda_joint1");
    let p2 = get_point(robot,"panda_joint3");
    let p3 = get_point(robot,"panda_joint4");
    let p4 = get_point(robot,"panda_joint6");
    let p5 = get_point(robot,"panda_joint7");
    let p6 = get_point(robot,"panda_gripper_joint");
    let robot_occ = Polyline::new(vec![p1,p2,p3,p4,p5],None);
    let robot_coll = Polyline::new(vec![p1,p2,p3,p4,p5,p6],None);

    let cost = |u: &[f64], c: &mut f64| {
        let c1 =  100.*drone_view_cost(&u,&destination);
        let c2 = 1.*drone_distance_cost(&u,&destination,&velocity);
        let c3 = 10.*drone_altitude_cost(&u,&destination);
        let c4 = 2.*drone_angle_cost(&u,&destination, &orientation);
        let (c5,c6) = drone_plane_cost(&u,&destination,&planes); //occlusion cost, safety cost
        let c7 = 20.*drone_safety(&u,&destination,&robot_coll);
        let c8 = 20.*drone_robot_occlusion(&u,&destination,&robot_occ);
        let c9 = drone_movement_cost(&u, &drone_current, &lb, &ub);
        let k = 1500.;
        *c=c1+c2+c3+c4+k*c5+c6+c7+c8+c9;
        Ok(())
    };

    let dcost = |u: &[f64], grad: &mut [f64]| {
        drone_finite_difference(&cost, u, grad)
    };

    let mut cur_cost = 0.0;
    cost(&drone_state, &mut cur_cost).unwrap();
    let problem = Problem::new(&bounds, dcost, cost);
    let mut opt = PANOCOptimizer::new(problem, panoc_cache).with_max_iter(50);//.with_max_duration(std::time::Duration::from_millis(10));

    bounds.project(&mut drone_state);
    let _status = opt.solve(&mut drone_state);
    if _status.is_err(){
        errors[2]=true;
    }
    else{
        if _status.unwrap().exit_status() == ExitStatus::NotConvergedOutOfTime {
            errors[3];
        }
        else{}
    }
    drone_state
}