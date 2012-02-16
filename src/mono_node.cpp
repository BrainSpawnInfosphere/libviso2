/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

/*
 The demo program was modified to run in ROS
 
 14 Feb 2012 Created Kevin J. Walchko
*/

// C++
#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h> // handles raw or compressed images
#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>

// Lib Viso2
#include <libviso2/viso_mono.h>

// OpenCV
#include <opencv2/opencv.hpp>

/**
 * This node uses the libviso2 to track points through successive images and 
 * determine the pose (position and orientation) of the camera. This demo node
 * shows simple ROS integration.
 */
class VisoOdometryMonoNode {
public:

    /**
     * Constructor
     */
    VisoOdometryMonoNode(ros::NodeHandle& node, std::string& s) : transport(node) {
        pose = Matrix::eye(4);
        viso = NULL;
        debug = false;
        
        // setup subscribers and publishers
        image_sub = transport.subscribe(s.c_str(), 1, &VisoOdometryMonoNode::imageCallback, this);
        pose_pub = node.advertise<geometry_msgs::Pose>("pose",10);
    }
    
    /**
     * Allow user to set default parameters
     */
    inline void set(const VisualOdometryMono::parameters& p){
        param = p;
    }
    
    inline void setDebug(const bool b){
        debug = b;
    }
    
    void print(){
        // output some statistics
        double num_matches = viso->getNumberOfMatches();
        double num_inliers = viso->getNumberOfInliers();
        std::cout << ", Matches: " << num_matches;
        std::cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << std::endl;
        std::cout << pose << std::endl << std::endl;
    }
		
private:
    /**
     * Create mono camera visual odometry using either a default param
     * or one passed to us by the set() function
     */
    void init(){
        viso = new VisualOdometryMono(param);
    }

    /**
     * Grabs image, processes it, and publishes results
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg){
		
		cv_bridge::CvImagePtr cv_msg;
		
		try{
			cv_msg = cv_bridge::toCvCopy(msg, "bgr8");
		}
		catch (sensor_msgs::CvBridgeException& error){
			ROS_ERROR("error");
			return;
		}
		
		// if viso has not been created in init() yet, then 
		// do so now
		if( viso == NULL){
		    init();
		    dims[0] = cv_msg->image.cols;
		    dims[1] = cv_msg->image.rows;
		    dims[2] = cv_msg->image.cols;
		}
		
		// convert BGR image into Gray image
		cv::Mat image;
		cv::cvtColor(cv_msg->image,image,CV_BGR2GRAY);
		
		if(debug){
		    cv::imshow("debug",image);
		    cv::waitKey(1);
        }
        
        // find motion
		bool ok = viso->process(image.data,dims);
		
		// update pose and publish only if process() was successful
		if(ok){
		    pose = pose * Matrix::inv(viso->getMotion());
            publish();
		}
	}
	
	/**
	 * Breaks up the pose matrix into a position vector and an 
	 * rotation quaternion and then publishes a Pose message
	 */
    bool publish(){
        // break pose down into position and quaternion
        bool ok = false;
        
        //print();
        
        geometry_msgs::Pose pose_msg;
        pose_msg.position.x = pose.val[0][3];
        pose_msg.position.y = pose.val[1][3];
        pose_msg.position.z = pose.val[2][3];
        
        pose_pub.publish(pose_msg);
        
        ok = true;
        
        return ok;
    }

    VisualOdometryMono::parameters param;
    VisualOdometryMono *viso;
    Matrix pose;
    int32_t dims[3];
    
	image_transport::ImageTransport transport;
	image_transport::Subscriber image_sub;
	ros::Publisher pose_pub;
	
	bool debug;
};

int main (int argc, char** argv) {
    
    ros::init(argc, argv, "viso_node");
    ros::NodeHandle n("~");
    
    
    // need the topic
    if (argc<2) {
        std::cerr << "Usage: ./mono_node topic_name " << std::endl;
        return 1;
    }
    
    // set most important visual odometry parameters
    // for a full parameter list, look at: viso_mono.h
    VisualOdometryMono::parameters param;
    param.calib.f  = 645.24; // focal length in pixels
    param.height = 1.0; // m
    
    std::string topic = argv[1];
    
    // create node
    VisoOdometryMonoNode viso(n,topic);
    viso.set(param);
    viso.setDebug(true);
    
    ros::spin();
    
    // exit
    return 0;
}

