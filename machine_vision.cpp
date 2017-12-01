/******************************************************
 * Author: Edgar Buchanan									          	*
 * File: machine_vision.cpp										        *
 * This file includes the main loop for the program.	*
 * Initializes tracking system, bluetooth and robots.	*
 * Checks the state of each robot.								    *
 ******************************************************/

#include <iostream>
#include <iomanip>
#include <vector>
#include <iCVCDriver.h>
#include <iCVCImg.h>
#include <iCVGenApi.h>
#include <CVCError.h>
#include <iCVCUtilities.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <ctime>

// Write files
#include <iostream>
#include <fstream>

#include <math.h>

// Actual project headers
#include "aruco_configuration.h"
#include "machine_vision.h"

using namespace std;
using namespace cv;

static const size_t DRIVERPATHSIZE = 256;

// Main loop
int main(int argc, char* argv[])
{
  // Open file
  ofstream myfile;
  myfile.open ("example.csv");

  // Header for file
  myfile 	<< "ID" << ","
		  << "X" << "," 
		  << "Y" << ","
		  << "Angle" << std::endl;

  // Variable used to retrieve coordinates
  tagData robot_coordinates;	

  bool b_activation_flag = false; // If true start logging
  int i_tag_id = 0; // Tag ID to be tracked

  // Capture images
  int i_Image_Counter = 0;

  std::cout.precision(3);

  /*****Camera calibration parameters**********/
  /*****Done on 31/08/2016 *********/
  /*https://github.com/daneshtarapore/apriltags-cpp/blob/optimisation/out_camera_data.xml*/

  double tmp_cameraMatrix[3][3] = {{1.6523377095739027e+03, 0.0, 7.9950000000000000e+02}, {0.0, 1.6523377095739027e+03, 7.9950000000000000e+02}, {0.0, 0.0, 1}};
  cv::Mat cameraMatrix = Mat(3, 3, CV_64FC1, &tmp_cameraMatrix);
  double tmp_distCoeffs[5][1] = {-1.9494404472059521e-01, 2.9965832643639467e-01, 0.0, 0.0, -3.4329528058097419e-01};
  cv::Mat distCoeffs = Mat(5, 1, CV_64FC1, &tmp_distCoeffs);
  /********************************************/

  IMG hCamera = initialize_machine_vision();
  // Start grab with ring buffer
  cvbres_t result = G2Grab(hCamera);

  if(result >= 0)
  {
      while(true)
      {
        // Wait for next image to be acquired
        // (returns immediately if unprocessed images are in the ring buffer)
        result = G2Wait(hCamera);

        if(result < 0)
            cout << setw(3) << " Error with G2Wait: " << CVC_ERROR_FROM_HRES(result) << endl;
        else
        {
			    // Create an attached OpenCV image
			    Mat distorted_image = cvb_to_ocv_nocopy(hCamera);

			    // Swap blue and red channels
			    vector<Mat> channels(3);
			    split(distorted_image, channels);
			    merge(vector<Mat>{channels[2], channels[1], channels[0]}, distorted_image);

			    // Undistort the image
			    Mat image;
			    // undistort(distorted_image, image, cameraMatrix, distCoeffs);
			    distorted_image.copyTo(image);
			
			    robot_coordinates = retrieve_position(cameraMatrix, distCoeffs, image, i_tag_id, robot_coordinates);
			
			    // Check for user input
			    int k = cvWaitKey(1) % 256;

			    if(k == 27) // Escape key
				    break; // Stop tracking

			    if(k == 10) // Enter key
			    {
				    b_activation_flag = true; // Start logging 
			    }
	
			    if(b_activation_flag) // Log data to csv file
			    {			
					    myfile << i_tag_id << ","
					    << robot_coordinates.x_pos << "," 
					    << robot_coordinates.y_pos << ","
					    << robot_coordinates.angle << std::endl;
			    }	

			    resize(image, image, Size(image.cols * 0.25, image.rows * 0.25), 0, 0, INTER_CUBIC);
		            imshow("JAI GO-5000C-PGE", image);				
		
			    /*
			     * Log to terminal
			     */
			    std::cout 	<< " ID = " << i_tag_id   
						    << " X = " << robot_coordinates.x_pos 
						    << " Y = " << robot_coordinates.y_pos
						    << " Angle = " << robot_coordinates.angle << std::endl;
		    }
      }
      cout << "Stop acquisition" << endl;

      // Stop the grab (kill = false: wait for ongoing frame acquisition to stop)
      result = G2Freeze(hCamera, true);
  }
  else
      cout << "Error starting acquisition!" << endl;
  // Free camera
  cout << "Free camera" << endl;
  ReleaseObject(hCamera);

  return 0;
}

