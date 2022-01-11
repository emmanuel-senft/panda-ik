#include "PandaIKRust.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf2_ros/transform_listener.h>
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
    commandedPose.pose.position.x=.3;
    commandedPose.pose.position.z=.3;
    commandedPose.pose.orientation.w=.1;

    geometry_msgs::Twist commandedVel = geometry_msgs::Twist();

    std::string frame_id = "panda_gripper_joint";
    bool initialized = false;

    int freq = 100;
    ros::Subscriber sub = nh.subscribe<geometry_msgs::TwistStamped>("input", 1,
        [&](const geometry_msgs::TwistStamped::ConstPtr& msg) {
            commandedVel = msg->twist;
            frame_id = msg->header.frame_id;
        }
    );
    ros::Rate loop_rate(freq);
    while (ros::ok()){
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
        
        tf2::Quaternion q(droneTransform.transform.rotation.x,droneTransform.transform.rotation.y,droneTransform.transform.rotation.z,droneTransform.transform.rotation.w);
        tf2::Matrix3x3 rot_drone = tf2::Matrix3x3(q);
        double r, p, y;
        rot_drone.getRPY(r, p, y, 2);

        std::string name = "panda_gripper_joint";//msg->child_frame_id;

        commandedPose.pose.position.x+=commandedVel.linear.x/freq;
        commandedPose.pose.position.y+=commandedVel.linear.y/freq;
        commandedPose.pose.position.z+=commandedVel.linear.z/freq;

        KDL::Rotation robot_rot = KDL::Rotation::Quaternion(commandedPose.pose.orientation.x,commandedPose.pose.orientation.y,commandedPose.pose.orientation.z,commandedPose.pose.orientation.w);
        KDL::Rotation motion = KDL::Rotation::RPY(commandedVel.angular.x/freq,commandedVel.angular.y/freq,commandedVel.angular.z/freq);
        (motion*robot_rot).GetQuaternion(commandedPose.pose.orientation.x,commandedPose.pose.orientation.y,commandedPose.pose.orientation.z,commandedPose.pose.orientation.w);

        std::array<double, 3> position = {commandedPose.pose.position.x, commandedPose.pose.position.y, commandedPose.pose.position.z};
        std::array<double, 4> orientation = {commandedPose.pose.orientation.x, commandedPose.pose.orientation.y, commandedPose.pose.orientation.z, commandedPose.pose.orientation.w};
        std::array<double, 3> velocity = {commandedVel.linear.x, commandedVel.linear.y,commandedVel.linear.z};
        
        std::array<double, 11> state = {joint_angles[0],joint_angles[1],joint_angles[2],joint_angles[3],joint_angles[4],joint_angles[5],joint_angles[6],droneTransform.transform.translation.x, droneTransform.transform.translation.y, droneTransform.transform.translation.z, y};

        solve(state.data(), name.c_str(), position.data(), orientation.data(), velocity.data());

        commandedPose.header.stamp = ros::Time::now();
        panda_pub.publish(commandedPose);
        // Confirm optimization has returned valid output (no nan -- happens occasionally)
        bool valid_output = true;

        for(int ii=0;ii<11;ii++){
            if(isnan(state[ii])){
                valid_output = false;
            }
        }

        if(initialized && valid_output){
            for(int ii=0;ii<7;ii++){
                if(fabs(state[ii]-joint_angles[ii]>.1)){
                    std::cout<<"Error"<<std::endl;
                    valid_output = false;
                }
            }
            if(!valid_output){
                if(fabs(commandedPose.pose.position.x - position[0])<.05 && fabs(commandedPose.pose.position.y - position[1])<.05 && fabs(commandedPose.pose.position.z - position[2])<.05){
                    valid_output = true;
                    std::cout<<"corrected"<<std::endl;
                }
            }
        }

        if(valid_output){
            initialized = true;
            auto joint_msg = std_msgs::Float64MultiArray();
            joint_msg.data.assign(state.data(), state.data() + 7);
            joint_angles = {state[0], state[1], state[2], state[3], state[4], state[5], state[6]};
            auto now = std::chrono::system_clock::now();
            while ((now - last_msg_time) < minimum_msg_delay) {
                now = std::chrono::system_clock::now();
            }
            last_msg_time = now;

            q.setEuler(0,0,state[10]);
            pub.publish(joint_msg);
            auto drone_msg = geometry_msgs::PoseStamped();
            drone_msg.header.frame_id = "panda_link0";
            drone_msg.header.stamp = ros::Time(0);
            drone_msg.pose.position.x=state[7];
            drone_msg.pose.position.y=state[8];
            drone_msg.pose.position.z=state[9];
            drone_msg.pose.orientation.x=q[0];
            drone_msg.pose.orientation.y=q[1];
            drone_msg.pose.orientation.z=q[2];
            drone_msg.pose.orientation.w=q[3];
            drone_pub.publish(drone_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 1;
}