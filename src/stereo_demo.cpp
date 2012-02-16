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
  Documented C++ sample code of stereo visual odometry (modify to your needs)
  To run this demonstration, download the Karlsruhe dataset sequence
  '2010_03_09_drive_0019' from: www.cvlibs.net!
  Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
*/

#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>

#include <viso_stereo.h>
#include <opencv2/opencv.hpp>

using namespace std;

int main (int argc, char** argv) {

  // we need the path name to 2010_03_09_drive_0019 as input argument
  if (argc<2) {
    cerr << "Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019" << endl;
    return 1;
  }

  // sequence directory
  string dir = argv[1];
  
  // set most important visual odometry parameters
  // for a full parameter list, look at: viso_stereo.h
  VisualOdometryStereo::parameters param;
  
  // calibration parameters for sequence 2010_03_09_drive_0019 
  param.calib.f  = 645.24; // focal length in pixels
  param.calib.cu = 661.96; // principal point (u-coordinate) in pixels
  param.calib.cv = 194.13; // principal point (v-coordinate) in pixels
  param.base     = 0.5707; // baseline in meters
  
  // init visual odometry
  VisualOdometryStereo viso(param);
  
  // current pose (this matrix transforms a point from the current
  // frame's camera coordinates to the first frame's camera coordinates)
  Matrix pose = Matrix::eye(4);
  
  cv::namedWindow("left",0);
  cv::namedWindow("right",0);
    
  // loop through all frames i=0:372
  for (int32_t i=0; i<373; i++) {

    // input file names
    char base_name[256]; sprintf(base_name,"%06d.png",i);
    string left_img_file_name  = dir + "/I1_" + base_name;
    string right_img_file_name = dir + "/I2_" + base_name;
    
    // catch image read/write errors here
    try {

      // load left and right input image
      cv::Mat left_img = cv::imread(left_img_file_name.c_str(),0); // load as black and white image
      cv::Mat right_img = cv::imread(right_img_file_name.c_str(),0);
      
      cv::imshow("left",left_img);
      cv::imshow("right",right_img);

      // image dimensions
      int32_t width = left_img.cols;  
      int32_t height = left_img.rows;

      // get pointers to the image data
      uint8_t* left_img_data  = left_img.data;
      uint8_t* right_img_data = right_img.data;
      
      // status
      cout << "Processing: Frame: " << i;
      
      // compute visual odometry
      int32_t dims[] = {width,height,width};
      if (viso.process(left_img_data,right_img_data,dims)) {
      
        // on success, update current pose
        pose = pose * Matrix::inv(viso.getMotion());
      
        // output some statistics
        double num_matches = viso.getNumberOfMatches();
        double num_inliers = viso.getNumberOfInliers();
        cout << ", Matches: " << num_matches;
        cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
        cout << pose << endl << endl;

      } else {
        cout << " ... failed!" << endl;
      }

    // catch image read errors here
    } catch (...) {
      cerr << "ERROR: Couldn't read input files!" << endl;
      return 1;
    }
  }
  
  // output
  cout << "Demo complete! Exiting ..." << endl;

  // exit
  return 0;
}

