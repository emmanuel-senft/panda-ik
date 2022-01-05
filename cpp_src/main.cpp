#include "PandaIKRust.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <chrono>

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

    geometry_msgs::PoseStamped lastPose = geometry_msgs::PoseStamped();
    bool lastPoseUndefined = true;

    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("input", 1,
        [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {


            geometry_msgs::TransformStamped droneTransform;
            try{
                droneTransform = tfBuffer.lookupTransform("panda_link0", "drone",
                                    ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                return;
            }

            auto pose = msg->pose;
            if(lastPoseUndefined){
                lastPose = *msg;
                lastPoseUndefined = false;
            }
            std::string name = msg->header.frame_id;
            std::array<double, 3> position = {pose.position.x, pose.position.y, pose.position.z};
            std::array<double, 4> orientation = {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
            
            double dt = (msg->header.stamp-lastPose.header.stamp).toSec();
            std::array<double, 3> velocity = {(pose.position.x-lastPose.pose.position.x)/dt,(pose.position.y-lastPose.pose.position.y)/dt,(pose.position.z-lastPose.pose.position.z)/dt};
            lastPose = *msg;

            tf2::Quaternion q(droneTransform.transform.rotation.x,droneTransform.transform.rotation.y,droneTransform.transform.rotation.z,droneTransform.transform.rotation.w);
            tf2::Matrix3x3 rot_drone = tf2::Matrix3x3(q);
            double r, p, y;
            rot_drone.getRPY(r, p, y, 2);
            
            std::array<double, 11> state = {joint_angles[0],joint_angles[1],joint_angles[2],joint_angles[3],joint_angles[4],joint_angles[5],joint_angles[6],droneTransform.transform.translation.x, droneTransform.transform.translation.y, droneTransform.transform.translation.z, y};

            solve(state.data(), name.c_str(), position.data(), orientation.data(), velocity.data());

            // Confirm optimization has returned valid output (no nan -- happens occasionally)
            bool valid_output = true;

            for(int ii=0;ii<11;ii++){
                if(isnan(state[ii])){
                    valid_output = false;
                }
            }

            if(valid_output){
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
        }
    );

    ros::spin();
}