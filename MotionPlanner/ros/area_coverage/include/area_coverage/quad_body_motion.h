//
// Created by redwan on 12/2/20.
//

#ifndef AREA_COVERAGE_QUAD_BODY_MOTION_H
#define AREA_COVERAGE_QUAD_BODY_MOTION_H
#include <ros/ros.h>
#include <memory>
#include <sensor_msgs/JointState.h>
#include "receeding_horizon.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "controller.h"

class QuadBodyMotion
{
public:
    QuadBodyMotion(const string &name, int robot_id, float yaw, shared_ptr<ReceedingHorizon> rh) : rh(rh), name(name), robot_id(robot_id), yaw(yaw)
    {
        q = 0;
        quadrotorView = nh.advertise<sensor_msgs::JointState>(name + "joint_states", 1);
        marker_pub = nh.advertise<visualization_msgs::Marker>("projection", 10);
        timer = nh.createTimer(ros::Duration(0.03), &QuadBodyMotion::timerCallback, this);
        initialized = false;
    }

    vector<double> getState(int robotID)
    {
        auto p = rh->getCoord(robotID);
        return vector<double>{p.x, p.y, rh->yaw[robotID]};
    }

    /**
     *
     * @param state {x, y, theta, z}
     */
    void publishRobotCoord(const vector<double> &state)
    {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = name + "base_link_inertia";

        transformStamped.transform.translation.x = state[0];
        transformStamped.transform.translation.y = state[1];
        transformStamped.transform.translation.z = state[3];
        tf2::Quaternion q1;
        q1.setRPY(0, 0, state[2]);
        transformStamped.transform.rotation.x = q1.x();
        transformStamped.transform.rotation.y = q1.y();
        transformStamped.transform.rotation.z = q1.z();
        transformStamped.transform.rotation.w = q1.w();

        br_base.sendTransform(transformStamped);
        transformStamped.child_frame_id = name + "base_link";
        br_base_link.sendTransform(transformStamped);
        //        rh->enabled = false;
    }

    void update_controller(int robot_id)
    {
        if (!rh->enabled && !initialized)
            return;

        if (!initialized)
        {
            double kpRho = 2, kpAlpha = 2 * 1.6, kpBeta = 2 * 0.3, dt = 0.03, thres = 0.25;
            cntrl = make_unique<controller>(getState(robot_id), kpRho, kpAlpha, kpBeta, dt, thres);
        }

        auto p = rh->getCoord(robot_id);
        cntrl->set_points(p.x, p.y);

        cntrl->compute_control();
        auto state = cntrl->get_state();
        state.push_back(p.z);

        ROS_INFO("[Robot %d] x = %lf y = %lf", robot_id, state[0], state[1]);
        publishRobotCoord(state);
        auto marker = getCone(state, robot_id);
        marker_pub.publish(marker);
        initialized = true;
    }

    void timerCallback(const ros::TimerEvent &event)
    {

        update_controller(robot_id);

        // update rotor
        // visualization update
        sensor_msgs::JointState Smsg;
        q = fmod((q + q_incr), (2.0 * M_PI));

        Smsg.header.stamp = ros::Time::now();
        for (int i = 0; i < 4; ++i)
        {
            std::string rotor_name = "rotor_" + std::to_string(i) + "_joint";
            if (i % 2 == 0)
                Smsg.position.push_back(q);
            else
                Smsg.position.push_back(-q);
            Smsg.name.push_back(rotor_name);
        }
        quadrotorView.publish(Smsg);
    }

private:
    const float q_incr = M_PI_4 / 2.0;
    float q = 0, yaw = 0;
    ros::Publisher quadrotorView;
    ros::Timer timer;
    ros::NodeHandle nh;
    shared_ptr<ReceedingHorizon> rh;
    unique_ptr<controller> cntrl;
    tf2_ros::TransformBroadcaster br_base, br_base_link;
    ros::Publisher marker_pub;
    const string name = "";
    int robot_id = 0;
    bool initialized = false;

protected:
    visualization_msgs::Marker getCone(const vector<double> &state, int id)
    {
        visualization_msgs::Marker cone;
        cone.header.stamp = ros::Time::now();
        cone.header.frame_id = "world";
        cone.action = visualization_msgs::Marker::ADD;
        cone.type = visualization_msgs::Marker::CUBE;
        cone.scale.x = cone.scale.y = 1;
        cone.scale.z = 0.002;
        cone.pose.position.x = state[0];
        cone.pose.position.y = state[1];
        cone.pose.position.z = -0.001;

        cone.pose.orientation.x = 0;
        cone.pose.orientation.y = 0;
        cone.pose.orientation.z = 0;
        cone.pose.orientation.w = 1;
        cone.color.a = 0.1;
        cone.color.g = 1.0;
        cone.color.r = 1.0;
        cone.id = id;
        cone.ns = "projection";
        return cone;
    }
};
#endif // AREA_COVERAGE_QUAD_BODY_MOTION_H
