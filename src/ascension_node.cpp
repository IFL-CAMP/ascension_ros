#include <PointATC3DG.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>

#include <iostream>
#include <algorithm>

double deg2rad (double degrees) {
    return degrees * 4.0 * atan (1.0) / 180.0;
}

int main(int argc, char **argv)
{    
    ros::init(argc, argv, "ascension_node");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // parse arguments
    
    std::string product_string; 
    nh.param<std::string>("product_id", product_string, "drivebay2");
    
    std::string base_frame_id; 
    nh.param<std::string>("base_frame_id", base_frame_id, "base");

    std::string root_frame_id;
    nh.param<std::string>("root_frame_id", root_frame_id, "base");
    
    std::vector<std::string> target_frame_ids;
    nh.param< std::vector<std::string> >("target_frame_ids", target_frame_ids, std::vector<std::string>());
    
    BirdProduct product = DRIVEBAY2;
    if (product_string == "drivebay2")
        product = DRIVEBAY2;
    else if (product_string == "medsafe")
        product = MEDSAFE;
    else if (product_string == "trakstar")
        product = TRAKSTAR;
    else
        throw std::runtime_error("Unrecognized device identifier! (must be one of [drivebay2,medsafe,trakstar])");
    
    // configure ATC3DG
    
    PointATC3DG bird(product);
    if (!bird) return -1;
    bird.setSuddenOutputChangeLock(0);
    
    int sensors_number = bird.getNumberOfSensors();
    ROS_INFO_STREAM("Detected " << sensors_number << " sensors");
    
    for (int i = 0; i < sensors_number; i++)
        bird.setSensorQuaternion(i);
    
    ROS_INFO_STREAM("The following target IDs will be used:");
    for (int i = 0; i < sensors_number; i++)
        ROS_INFO_STREAM("Sensor " << i+1 << ": " << target_frame_ids[i]);
    
    // create publishers
    
    ros::Publisher pub = nh.advertise<geometry_msgs::TransformStamped>("target_poses", 10);
    tf::TransformBroadcaster br;

    ros::Rate rate(60);
    tf::StampedTransform t;
    while (ros::ok())
    {
        t.stamp_ = ros::Time::now();
        for(int i = 0; i < sensors_number; ++i) 
        {
            double dX, dY, dZ, quat[4];
            bird.getCoordinatesQuaternion(i, dX, dY, dZ, quat);

            t.frame_id_ = base_frame_id;
            t.child_frame_id_ = target_frame_ids[i];
            t.setOrigin(tf::Vector3(dX, dY, dZ));
            t.setRotation(tf::Quaternion(-quat[1], -quat[2], -quat[3], quat[0]));

            if (root_frame_id == target_frame_ids[i]) {
                t.frame_id_ = target_frame_ids[i];
                t.child_frame_id_ = base_frame_id;
                t.setData(t.inverse());
                br.sendTransform(t);

                geometry_msgs::TransformStamped msg;
                tf::transformStampedTFToMsg(t, msg);
                pub.publish(msg);
            } else {
                br.sendTransform(t);

                geometry_msgs::TransformStamped msg;
                tf::transformStampedTFToMsg(t, msg);
                pub.publish(msg);
            }

        }

        rate.sleep();
    }
    
    return 0;
}