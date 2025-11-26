//
// Created by redwan on 12/2/20.
//
#include <ros/ros.h>
#include "area_coverage/quad_body_motion.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "coverage_view");
    ros::NodeHandle nh;
    ROS_INFO("Coverage view node started");
    auto rh = make_shared<ReceedingHorizon>();
    double q0, q1, q2;
    nh.getParam("q0", q0);
    nh.getParam("q1", q1);
    nh.getParam("q2", q2);
    auto body = new QuadBodyMotion("uav/", 0, q0, rh->getPtr());
    auto body1 = new QuadBodyMotion("uav1/", 1, q1, rh->getPtr());
    auto body2 = new QuadBodyMotion("uav2/", 2, q2, rh->getPtr());
    // Suppress unused variable warnings
    (void)body;
    (void)body1;
    (void)body2;
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    // cleanup
    delete body;
    delete body1;
    delete body2;
    return 0;
}