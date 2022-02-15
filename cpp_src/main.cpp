#include <stdint.h>
#include "PandaIKRust.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64MultiArray.h"
#include "drone_ros_msgs/Planes.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <kdl/kdl.hpp>
#include <iostream>
#include <chrono>
#include <math.h>

std::array<double, 7> joint_angles = {0,0,0,-1.5,0,1.5,0};
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "panda_ik");
    ros::NodeHandle nh("~");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    auto get_required_param = [&](const std::string k, auto& v) {
        if (!nh.getParam(k, v)) {
            std::string msg = "Could not find required parameter: " + k;
            ROS_ERROR(
                msg.c_str()
            );
            return false;
        }
        return true;
    };

    std::string urdf;
    if (!get_required_param("URDF", urdf)) return 0;
    if (!init(urdf.c_str())) return 0;


    std::chrono::system_clock::time_point last_msg_time = std::chrono::system_clock::now();
    std::chrono::milliseconds minimum_msg_delay = std::chrono::milliseconds(0);

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("output", 1);
    ros::Publisher drone_pub = nh.advertise<geometry_msgs::PoseStamped>("drone_output", 1);
    ros::Publisher panda_pub = nh.advertise<geometry_msgs::PoseStamped>("panda_commanded_pose", 1);

    geometry_msgs::PoseStamped commandedPose = geometry_msgs::PoseStamped();
    commandedPose.header.frame_id = "panda_link0";
    commandedPose.pose.position.x=.4;
    commandedPose.pose.position.z=.4;
    commandedPose.pose.orientation.w=.1;

    geometry_msgs::Twist commandedVel = geometry_msgs::Twist();

    geometry_msgs::PoseStamped droneCommandedPose = geometry_msgs::PoseStamped();
    bool reaching_drone_pose=false;

    std::string frame_id = "panda_gripper_joint";
    bool initialized = false;
    bool drone_init = false;

    int freq = 100;
    bool start = false;
    ros::Subscriber twistSub = nh.subscribe<geometry_msgs::TwistStamped>("input", 1,
        [&](const geometry_msgs::TwistStamped::ConstPtr& msg) {
            commandedVel = msg->twist;
            frame_id = msg->header.frame_id;
            start = true;
        }
    );
    ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseStamped>("drone_goal", 1,
        [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
            droneCommandedPose = *msg;
            reaching_drone_pose = true;
            cout<<"Got pose"<<endl;
        }
    );
    vector<double> normals;
    vector<double> points;
    vector<double> centers;
    vector<double> orientations;
    vector<double> half_axes;

    uint8_t plane_numbers = 0;


    ros::Subscriber planeSub = nh.subscribe<drone_ros_msgs::Planes>("planes", 1,
        [&](const drone_ros_msgs::Planes::ConstPtr& msg) {
            normals.clear();
            points.clear();
            centers.clear();
            orientations.clear();
            half_axes.clear();
            plane_numbers=msg->normals.size();
            for(geometry_msgs::Point normal: msg->normals){
                normals.push_back(normal.x);
                normals.push_back(normal.y);
                normals.push_back(normal.z);
            }
            for(geometry_msgs::Pose pose: msg->poses){
                centers.push_back(pose.position.x);
                centers.push_back(pose.position.y);
                centers.push_back(pose.position.z);
                orientations.push_back(pose.orientation.x);
                orientations.push_back(pose.orientation.y);
                orientations.push_back(pose.orientation.z);
                orientations.push_back(pose.orientation.w);
            }
            for(geometry_msgs::Vector3 half_axe: msg->half_axes){
                half_axes.push_back(half_axe.x);
                half_axes.push_back(half_axe.y);
                half_axes.push_back(half_axe.z);
            }
            for(geometry_msgs::Polygon poly: msg->planes){
                for(geometry_msgs::Point32 p: poly.points){
                    points.push_back(p.x);
                    points.push_back(p.y);
                    points.push_back(p.z);
                }
            }
        }
    );
    std::array<double, 4> last_drone_goal = {0,0,0,0};
    ros::Rate loop_rate(freq);
    while (ros::ok()){
        if(! start){
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        geometry_msgs::TransformStamped droneTransform;
        try{
            droneTransform = tfBuffer.lookupTransform("panda_link0", "drone",
                                ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        std::array<double,3> uncertainty = {0.05,0.05,0.02};
        KDL::Rotation drone_rot = KDL::Rotation::Quaternion(droneTransform.transform.rotation.x,droneTransform.transform.rotation.y,droneTransform.transform.rotation.z,droneTransform.transform.rotation.w);
        double r, p, y;
        drone_rot.GetRPY(r, p, y);
        std::array<double, 4> drone_current = {droneTransform.transform.translation.x,droneTransform.transform.translation.y,droneTransform.transform.translation.z,y};
        
        if(!drone_init){
            drone_init = true;
            last_drone_goal = drone_current;
        }

        bool valid_output = true;
        std::string name = "panda_gripper_joint";//msg->child_frame_id;

        std::array<bool, 4> errors = {0,0,0,0};

        std::array<double, 7> robot_state = joint_angles;
        std::array<double, 3> position = {commandedPose.pose.position.x, commandedPose.pose.position.y, commandedPose.pose.position.z};
            
        std::array<double, 4> drone_goal = last_drone_goal;
        if(reaching_drone_pose){
            std::array<double, 4> drone_goal;
            KDL::Rotation drone_goal_rot = KDL::Rotation::Quaternion(droneCommandedPose.pose.orientation.x,droneCommandedPose.pose.orientation.y,droneCommandedPose.pose.orientation.z,droneCommandedPose.pose.orientation.w);
            double r, p, y;
            drone_goal_rot.GetRPY(r, p, y);
            drone_goal[0]=droneCommandedPose.pose.position.x;
            drone_goal[1]=droneCommandedPose.pose.position.y;
            drone_goal[2]=droneCommandedPose.pose.position.z;
            drone_goal[3]=y;
            double threshold = .1;
            reaching_drone_pose=false;
            for(int i=0;i<4;i++){
                if(pow(drone_goal[i]-drone_current[i],2)>threshold)
                    reaching_drone_pose = true;
            }
            if(reaching_drone_pose)
                solveDroneOnly(robot_state.data(), drone_current.data(), drone_goal.data(), last_drone_goal.data(), errors.data(), 
            &normals[0], &points[0], &centers[0], &orientations[0], &half_axes[0], &plane_numbers, &uncertainty[0]);
        }
        if(!reaching_drone_pose){
            commandedPose.pose.position.x+=commandedVel.linear.x/freq;
            commandedPose.pose.position.y+=commandedVel.linear.y/freq;
            commandedPose.pose.position.z+=commandedVel.linear.z/freq;
            
            KDL::Rotation robot_rot = KDL::Rotation::Quaternion(commandedPose.pose.orientation.x,commandedPose.pose.orientation.y,commandedPose.pose.orientation.z,commandedPose.pose.orientation.w);
            KDL::Rotation motion = KDL::Rotation::RPY(commandedVel.angular.x/freq,commandedVel.angular.y/freq,commandedVel.angular.z/freq);
            (motion*robot_rot).GetQuaternion(commandedPose.pose.orientation.x,commandedPose.pose.orientation.y,commandedPose.pose.orientation.z,commandedPose.pose.orientation.w);
            
            double norm = sqrt(commandedPose.pose.orientation.x*commandedPose.pose.orientation.x+commandedPose.pose.orientation.y*commandedPose.pose.orientation.y+commandedPose.pose.orientation.z*commandedPose.pose.orientation.z+commandedPose.pose.orientation.w*commandedPose.pose.orientation.w);
            if (norm != 1){
                //cout<<"Renormalization"<<endl;
                commandedPose.pose.orientation.x /= norm;
                commandedPose.pose.orientation.y /= norm;
                commandedPose.pose.orientation.z /= norm;
                commandedPose.pose.orientation.w /= norm;
            }

        //robot error, robot out of time, drone error, drone out of time
            std::array<double, 4> orientation = {commandedPose.pose.orientation.x, commandedPose.pose.orientation.y, commandedPose.pose.orientation.z, commandedPose.pose.orientation.w};
            std::array<double, 3> velocity = {commandedVel.linear.x, commandedVel.linear.y,commandedVel.linear.z};
        
            solve(robot_state.data(), drone_current.data(), last_drone_goal.data(), name.c_str(), position.data(), orientation.data(), 
              velocity.data(), errors.data(), &normals[0], &points[0], &centers[0], &orientations[0], &half_axes[0], &plane_numbers, &uncertainty[0]);
        }

        if(errors[0]){
            robot_state=joint_angles;
        }
        if(errors[1]){
            ;
        }
        if(errors[2]){
            std::cout<<"drone error"<<std::endl;
            drone_current=last_drone_goal;
        }
        if(errors[3]){
            drone_current=last_drone_goal;
            std::cout<<"drone timeout"<<std::endl;
        }
        commandedPose.header.stamp = ros::Time::now();
        panda_pub.publish(commandedPose);

        if(initialized && valid_output){
            for(int ii=0;ii<7;ii++){
                if(fabs(robot_state[ii]-joint_angles[ii]>.1)){
                    valid_output = false;
                }
            }
            if(!valid_output){
               if(fabs(commandedPose.pose.position.x - position[0])<.05 && fabs(commandedPose.pose.position.y - position[1])<.05 && fabs(commandedPose.pose.position.z - position[2])<.05){
                   valid_output = true;
               }
            }
        }
        if(!valid_output)
            robot_state=joint_angles;

        initialized = true;
        auto joint_msg = std_msgs::Float64MultiArray();
        joint_msg.data.assign(robot_state.data(), robot_state.data() + 7);
        joint_angles = robot_state;

        tf2::Quaternion q(0,0,0,1);

        q.setEuler(0,0,drone_current[3]);
        auto drone_msg = geometry_msgs::PoseStamped();
        drone_msg.header.frame_id = "panda_link0";
        drone_msg.header.stamp = ros::Time(0);
        drone_msg.pose.position.x=drone_current[0];
        drone_msg.pose.position.y=drone_current[1];
        drone_msg.pose.position.z=drone_current[2];
        drone_msg.pose.orientation.x=q[0];
        drone_msg.pose.orientation.y=q[1];
        drone_msg.pose.orientation.z=q[2];
        drone_msg.pose.orientation.w=q[3];

        auto now = std::chrono::system_clock::now();
        pub.publish(joint_msg);
        drone_pub.publish(drone_msg);

        while ((now - last_msg_time) < minimum_msg_delay) {
            now = std::chrono::system_clock::now();
        }
        last_msg_time = now;
        last_drone_goal = drone_current;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 1;
}
