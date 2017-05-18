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
    
    // parse arguments
    
    std::string product_string; 
    nh.param<std::string>("product_id", product_string, "drivebay2");
    
    std::string base_frame_id; 
    nh.param<std::string>("base_frame_id", base_frame_id, "base");
    
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
    
    ROS_INFO_STREAM("The following target IDs will be used:");
    for (int i = 0; i < sensors_number; i++)
        ROS_INFO_STREAM("Sensor " << i+1 << ": " << target_frame_ids[i]);
    
    // create publishers
    
    ros::Publisher pub = nh.advertise<geometry_msgs::TransformStamped>("target_poses", 10);
    tf::TransformBroadcaster br;
    
    while (ros::ok())
    {
        for(int i = 0; i < sensors_number; ++i) 
        {
            double dX, dY, dZ, dAzimuth, dElevation, dRoll;
            bird.getCoordinatesAngles(i, dX, dY, dZ, dAzimuth, dElevation, dRoll);
            
            tf::StampedTransform t;
            t.frame_id_ = base_frame_id;
            t.child_frame_id_ = target_frame_ids[i];
            t.setOrigin(tf::Vector3(dX/100, dY/100, dZ/100));
            t.setRotation(tf::createQuaternionFromRPY(deg2rad(dAzimuth), deg2rad(dElevation), deg2rad(dRoll)));
            
            br.sendTransform(t);
            
            geometry_msgs::TransformStamped msg;
            tf::transformStampedTFToMsg(t, msg);
            pub.publish(msg);            
        }
    }
    
    return 0;
}