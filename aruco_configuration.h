/****************************************************************
 * Author: Edgar Buchanan										*
 * File: aruco_configuration.h									*
 * Functions to initialize tracking system and to recover		*
 * position and orientation of the tags.						*
 * Function to capture frames can be also found in this file.	*
 ****************************************************************/

#ifndef ARUCOCONF_H
#define ARUCOCONF_H

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

using namespace std;
using namespace cv;

#define ARENA_HEIGHT 1.0f // table: 75cm; Floor: 2cm
#define CAMERA_HEIGHT 245.75f
#define ROBOT_HEIGHT 4.0f // Psi-swarm

typedef struct{
	float x_pos;
	float y_pos;
	float angle;
	Point2f pixel_coord;
} tagData;

/*
 * Initialize machine vision
 */
IMG initialize_machine_vision();
Mat cvb_to_ocv_nocopy(IMG cvbImg);
void projectpoint_image_to_world(cv::Mat cameraMatrix, double world_z, cv::Point_<float> 	undistort_imgpoint, cv::Point_<float> &undistort_worldpoint);

/*
 * Capture in image actual frame
 * @param image - Actual frame
 * @param counter - Number of the frame
 * @return Increases counter by one.
 */
int capture_image(Mat image, int counter);

/*
 * Retrieves the position of a single tag
 * @param ID - ID of the tag requested
 * @param prev_pos - Previous position of the tag
 * @return Position of the marker of ID.
 */
tagData retrieve_position(cv::Mat cameraMatrix, cv::Mat distCoeffs, Mat image, int ID, tagData prev_pos);

#endif /* ARUCOCONF_H */
