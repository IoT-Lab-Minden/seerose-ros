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
 * PointerLocalizerRobot.h
 *
 *  Created on: 12.02.2018
 *      Author: dsprute
 */

#ifndef SRC_POINTERLOCALIZERROBOT_H_
#define SRC_POINTERLOCALIZERROBOT_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/image_transport.h>

class PointerLocalizerRobot
{
public:

	PointerLocalizerRobot();
	virtual ~PointerLocalizerRobot();

private:

	/**
	 * Parameter name of the input RGB image topic.
	 */
	static const std::string PARAM_INPUT_RGB_IMAGE_TOPIC;

	/**
	 * Parameter name of the input depth image topic.
	 */
	static const std::string PARAM_INPUT_DEPTH_IMAGE_TOPIC;

	/**
	 * Parameter name of the RGB camera info topic.
	 */
	static const std::string PARAM_RGB_CAMERA_INFO_TOPIC;

	/**
	 * Default name of the topic of the input RGB image.
	 */
	static const std::string TOPIC_INPUT_RGB_IMAGE_DEFAULT;

	/**
	 * Default name of the topic of the input depth image.
	 */
	static const std::string TOPIC_INPUT_DEPTH_IMAGE_DEFAULT;

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
	 * Typedef for synchronization policy.
	 */
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> t_syncPolicy;

	/**
	 * Name of the topic of the RGB input image.
	 */
	std::string m_topicInputRGBImage;

	/**
	 * Name of the topic of the depth input image.
	 */
	std::string m_topicInputDepthImage;

	/**
	 * Name of the topic of the RGB camera info.
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
	 * Subscriber for the RGB image.
	 */
	image_transport::SubscriberFilter m_rgbImageSubscriber;

	/**
	 * Subscriber for the depth image.
	 */
	image_transport::SubscriberFilter m_depthImageSubscriber;

	/**
	 * Subscriber for the depth camera info.
	 */
	ros::Subscriber m_rgbCameraInfoSubscriber;

	/**
	 * Publisher for the pointing point.
	 */
	ros::Publisher m_pointingPointPublisher;

	/**
	 * Publisher for the output image containing the laser point detection.
	 */
	image_transport::Publisher m_imagePublisher;

	/**
	 * Publisher for the output image containing the marker detections.
	 */
	//image_transport::Publisher m_debugImagePublisher;

	/**
	 * Synchronizer for image topics.
	 */
	message_filters::Synchronizer<t_syncPolicy> m_synchronizer;

	/**
	 * Intrinsic RGB camera matrix.
	 */
	cv::Mat m_intrinsicRGBCameraMatrix;

	/**
	 * Callback that is invoked if two synchronized images are received.
	 *
	 * @param f_rgbImage RGB input image.
	 * @param f_depthImage Depth input image.
	 *
	 */
	void imagesReceivedCallback(const sensor_msgs::ImageConstPtr& f_rgbImage, const sensor_msgs::ImageConstPtr& f_depthImage);

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
	 * Calculates the pose of the laser point with respect to the camera.
	 *
	 * @param f_depthImage The depth image containing depth information [in m] for each pixel.
	 * @param f_xScale The scale factor for f_depthImage in x-direction w.r.t. the RGB image.
	 * @param f_yScale The scale factor for f_depthImage in y-direction w.r.t. the RGB image.
	 * @param f_laserpointContour The contour of the laser point.
	 * @param f_pose The calculated pose of the laser point with respect to the camera.
	 */
	void projectInto3D(const cv::Mat& f_depthImage, float f_xScale, float f_yScale, const cv::Point& f_point2D,
			geometry_msgs::PointStamped& f_point3D) const;

	/**
	 * Draws the laser point (contour) into the image.
	 *
	 * @param f_rgbImage The RGB image to be drawn into.
	 * @param f_laserpoint The laser point to be drawn.
	 * @param f_color The color of the drawing.
	 */
	void drawLaserpoint(cv::Mat& f_rgbImage, const std::vector<cv::Point>& f_laserpoint, cv::Scalar f_color = cv::Scalar(0, 0, 255)) const;

	/**
	 * Converts a ROS image to an OpenCV Mat.
	 *
	 * @param f_image The ROS image.
	 * @param f_mat The OpenCV Mat.
	 * @param f_encoding The image encoding.
	 */
	void image2Mat(const sensor_msgs::Image& f_image, cv::Mat& f_mat, const std::string& f_encoding) const;
};

#endif /* SRC_POINTERLOCALIZERROBOT_H_ */

