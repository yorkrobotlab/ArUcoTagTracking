/****************************************************************
 * Author: Edgar Buchanan				                    						*
 * File: aruco_configuration.cpp								                *
 * Functions to initialize tracking system and to recover		    *
 * position and orientation of the tags.					            	*
 * Function to capture frames can be also found in this file.	  *
 ****************************************************************/

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

#include "aruco_configuration.h"


static const size_t DRIVERPATHSIZE = 256;

/*
 * Initialize machine vision
 */
IMG initialize_machine_vision()
{
	
    // Load the camera
    char driverPath[DRIVERPATHSIZE] = { 0 };
    TranslateFileName("%CVB%/drivers/GenICam.vin", driverPath, DRIVERPATHSIZE);
    IMG hCamera = NULL;

    bool success = LoadImageFile(driverPath, hCamera);

    if(!success)
    {
        cout << "Error loading " << driverPath << " driver!" << endl;
        // return 1;
    }
	else{
    	cout << "Load " << driverPath << " successful." << endl;
	}
    return hCamera;
}

/*
 * Capture in image actual frame
 * @param image - Actual frame
 * @param counter - Number of the frame
 * @return Increases counter by one.
 */
int capture_image(Mat image, int counter)
{
    //printf("Capturing image %d\n", counter);

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

    ostringstream os;
	  if(counter < 10)
		  os << "image_000000" << counter << ".png";
	  else if(counter < 100)
		  os << "image_00000" << counter << ".png";
	  else if(counter < 1000)
		  os << "image_0000" << counter << ".png";
	  else if(counter < 10000)
		  os << "image_000" << counter << ".png";
	  else if(counter < 100000)
		  os << "image_00" << counter << ".png";
	  else if(counter < 1000000)
		  os << "image_0" << counter << ".png";
	  else if(counter < 10000000)
		  os << "image_" << counter << ".png";

    imwrite(os.str(), image, compression_params);
    return ++counter;
}

/*
 * Retrieves the position of a single tag
 * @param ID - ID of the tag requested
 * @return Position of the marker of ID.
 */
tagData retrieve_position(cv::Mat cameraMatrix, cv::Mat distCoeffs, Mat image, int ID, tagData prev_pos){
  
  vector<int> marker_ids;
  vector<vector<Point2f> > marker_corners, rejected_candidates; // Vectors for ARuCo Corners
  Point2f projected_centres[9];
  
  tagData position_ret = prev_pos; // In case tag is not detected stored last position

  Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
  // Load dictionary
  Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_50);
  // Detect markers from image
  aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, parameters, rejected_candidates);
  // If ARuCo tag detected
	if(marker_ids.size() > 0)
	{
	  // For each tag
		for(size_t marker_index = 0; marker_index < marker_ids.size(); marker_index++)
		{
		  // For specified tag
			if(marker_ids[marker_index] == ID)
			{
				double bottom_line_angle = atan2(marker_corners[marker_index][2].y - marker_corners[marker_index][3].y,
							     marker_corners[marker_index][2].x - marker_corners[marker_index][3].x)
						       * 180.0f / M_PI; // atan2 returns angle in radians [-pi, pi]

				if(std::isnan(bottom_line_angle))
				{
					cout << " angle is nan for tag points " << marker_corners[marker_index] << endl;
					exit(-1);
				}

				Mat_<Point2f> points(1, 4);
				points(0) = marker_corners[marker_index][0];
				points(1) = marker_corners[marker_index][1];
				points(2) = marker_corners[marker_index][2];
				points(3) = marker_corners[marker_index][3];

				// Mat undistorted;
				Mat_<Point2f> undistorted(1, 4);
				undistortPoints(points, undistorted, cameraMatrix, distCoeffs, noArray(), cameraMatrix);

				Point2f undistorted_centre = (undistorted(0) +
				    undistorted(1) +
				    undistorted(2) +
				    undistorted(3)) / 4;
				
				double undistorted_angle = atan2(undistorted(2).y - undistorted(3).y,
							     undistorted(2).x - undistorted(3).x)
						       * 180.0f / M_PI; // atan2 returns angle in radians [-pi, pi]

				
				double world_z = CAMERA_HEIGHT - ROBOT_HEIGHT - ARENA_HEIGHT; // assume the camera is above the robot's tags

				cv::Point_<float> undistorted_projected_center;
				projectpoint_image_to_world(cameraMatrix, world_z, undistorted_centre, undistorted_projected_center);
				
				projected_centres[marker_ids[marker_index]] = undistorted_projected_center;
				
				// Store new position and orientation
				position_ret.x_pos = undistorted_projected_center.x;
				position_ret.y_pos = undistorted_projected_center.y;
				position_ret.angle = undistorted_angle;
				position_ret.pixel_coord = undistorted_centre; 			
			}	
		}
	}
	return position_ret;
}

Mat cvb_to_ocv_nocopy(IMG cvbImg)
{
    // Construct an appropriate OpenCV image
    Size size(ImageWidth(cvbImg), ImageHeight(cvbImg));
    void* ppixels = nullptr;
    intptr_t xInc = 0;
    intptr_t yInc = 0;
    GetLinearAccess(cvbImg, 0, &ppixels, &xInc, &yInc);
    Mat image(size, CV_8UC3, ppixels, yInc);

    return image;
}

void projectpoint_image_to_world(cv::Mat cameraMatrix, double world_z, cv::Point_<float> undistort_imgpoint, cv::Point_<float> &undistort_worldpoint)
{
    double image_x = undistort_imgpoint.x;
    double image_y = undistort_imgpoint.y;

    double c_x = cameraMatrix.at<double>(0,2);
    double c_y = cameraMatrix.at<double>(1,2);

    double f_x = cameraMatrix.at<double>(0,0);
    double f_y = cameraMatrix.at<double>(1,1);

    /*
     * ref: http://stackoverflow.com/questions/12007775/to-calculate-world-coordinates-from-screen-coordinates-with-opencv
     x_screen = (x_world/z_world)*f_x + c_x
     y_screen = (y_world/z_world)*f_y + c_y

     x_world = (x_screen - c_x) * z_world / f_x
     y_world = (y_screen - c_y) * z_world / f_y*/

    double world_x = (image_x - c_x) * world_z / f_x;
    double world_y = (image_y - c_y) * world_z / f_y;

    undistort_worldpoint.x = world_x;
    undistort_worldpoint.y = world_y;

    /*
     * But from my calculations, I think we should use the following:
    double world_x = (image_x - c_x * world_z) / f_x;
    double world_y = (image_y - c_y * world_z) / f_y;*/
}



