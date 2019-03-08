/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, IoT-Lab Minden
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
/*
 * PointerLocalizerIE.h
 *
 *  Created on: 03.01.2018
 *      Author: dsprute
 */

#ifndef SRC_POINTERLOCALIZERIE_H_
#define SRC_POINTERLOCALIZERIE_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/image_transport.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <lab_msgs/Plane.h>
#include <lab_msgs/Line.h>
#include <opencv2/video/video.hpp>

class PointerLocalizerIE
{
public:

	PointerLocalizerIE();
	virtual ~PointerLocalizerIE();

private:

	/**
	 * The frame id of the global/map coordinate frame.
	 */
	static const std::string TF_MAP;

	/**
	 * Parameter name of the input RGB image topic.
	 */
	static const std::string PARAM_INPUT_RGB_IMAGE_TOPIC;

	/**
	 * Parameter name of the rgb camera info topic.
	 */
	static const std::string PARAM_RGB_CAMERA_INFO_TOPIC;

	/**
	 * Default name of the topic of the input RGB image.
	 */
	static const std::string TOPIC_INPUT_RGB_IMAGE_DEFAULT;

	/**
	 * Default name of the topic of the rgb camera info.
	 */
	static const std::string TOPIC_RGB_CAMERA_INFO_DEFAULT;

	/**
	 * Name of the topic of the pointer output image.
	 */
	static const std::string TOPIC_POINTER_IMAGE;

	/**
	 * Name of the topic of the laser point.
	 */
	static const std::string TOPIC_POINTING_POINT;

	/**
	 * Name of the topic of the RGB input image.
	 */
	std::string m_topicInputRGBImage;

	/**
	 * Name of the topic of the rgb camera info.
	 */
	std::string m_topicRGBCameraInfo;

	/**
	 * Node handle.
	 */
	ros::NodeHandle m_nodeHandle;

	/**
	 * Image transporter.
	 */
	image_transport::ImageTransport m_imageTransporter;

	/**
	 * Subscriber for the input image.
	 */
	image_transport::Subscriber m_imageSubscriber;

	/**
	 * Subscriber for the RGB camera info.
	 */
	ros::Subscriber m_rgbCameraInfoSubscriber;

	/**
	 * Publisher for the pointing point.
	 */
	ros::Publisher m_pointingPointPublisher;

	/**
	 * Publisher for the output image.
	 */
	image_transport::Publisher m_imagePublisher;

	/**
	 * Publisher for the output image containing the marker detections.
	 */
	//image_transport::Publisher m_debugImagePublisher;

	/**
	 * Intrinsic RGB camera matrix.
	 */
	cv::Mat m_intrinsicRGBCameraMatrix;

	/**
	 * Last RGB image.
	 */
	sensor_msgs::Image m_lastRGBImage;

	/**
	 * Model of the ground plane.
	 */
	lab_msgs::Plane m_groundPlane;

	/**
	 * Listener to the transform tree (needed for coordinate frame transformations).
	 */
	tf::TransformListener m_tflistener;

	/**
	 * Background subtractor.
	 */
	cv::BackgroundSubtractorMOG2 m_backgroundSubtractor;

	/**
	 * Last laser spot position.
	 */
	std::vector<cv::Point> m_laserpoints;

	/**
	 * Callback that is invoked when a new input image is received.
	 * @param f_image Pointer to the input image.
	 */
	void imageReceivedCallback(const sensor_msgs::ImageConstPtr& f_image);

	/**
	 * Callback that is invoked when a new RGB camera info is received.
	 *
	 * @param f_cameraInfo Pointer to the camera info.
	 */
	void rgbCameraInfoReceivedCallback(const sensor_msgs::CameraInfo& f_cameraInfo);

	/**
	 * Detects a laser point in a given image.
	 *
	 * @param f_rgbImage The input RGB image.
	 * @param f_laserpoint The detected laser point if found.
	 * @return True if a laser point is detected in the image.
	 */
	bool detectLaserPoint(const cv::Mat& f_rgbImage, cv::Point& f_laserpoint);

	/**
	 * Draws the laser points (contours) into the image.
	 *
	 * @param f_rgbImage The RGB image to be drawn into.
	 * @param f_laserpoints The laser point to be drawn.
	 * @param f_color The color of the drawing.
	 */
	void drawLaserpoints(cv::Mat& f_rgbImage, const std::vector<cv::Point>& f_laserpoints,
			cv::Scalar f_color = cv::Scalar(0, 0, 255)) const;

	/**
	 * Converts a ROS image to an OpenCV Mat.
	 *
	 * @param f_image The ROS image.
	 * @param f_mat The OpenCV Mat.
	 * @param f_encoding The image encoding.
	 */
	void image2Mat(const sensor_msgs::Image& f_image, cv::Mat& f_mat, const std::string& f_encoding) const;

	/**
	 * Calculates the position of the given image point on the given plane.
	 * @param f_imagePoint The image coordinate.
	 * @param f_plane The plane.
	 * @param f_spacePoint The position of the given image point on the given plane. If the 3D position could not be calculated, all
	 * components of the point are set to -1.
	 * @return True on success.
	 */
	bool getPositionOnPlane(const cv::Point2f& f_imagePoint, const lab_msgs::Plane& f_plane,
			geometry_msgs::PointStamped& f_spacePoint) const;

	/**
	 * Calculates the intersection point between the given line and plane if available.
	 * @param f_plane The (ground) plane.
	 * @param f_line The line.
	 * @param f_intersectionPoint The intersection point between the given line and plane if available.
	 * @return True if an intersection could be calculated.
	 */
	bool getPlaneIntersection(const lab_msgs::Plane& f_plane, const lab_msgs::Line& f_line,
			geometry_msgs::PointStamped& f_intersectionPoint) const;
};

#endif /* SRC_POINTERLOCALIZERIE_H_ */

