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
 * PointerLocalizer.cpp
 *
 *  	12.02.2018
 *      Author: Dennis Sprute
 */

#include "pointer_detection/PointerLocalizerRobot.h"

const std::string PointerLocalizerRobot::PARAM_INPUT_RGB_IMAGE_TOPIC = "input_rgb_topic";
const std::string PointerLocalizerRobot::PARAM_INPUT_DEPTH_IMAGE_TOPIC = "input_depth_topic";
const std::string PointerLocalizerRobot::PARAM_RGB_CAMERA_INFO_TOPIC = "rgb_camera_info_topic";
const std::string PointerLocalizerRobot::TOPIC_INPUT_RGB_IMAGE_DEFAULT = "/camera/rgb/image_raw";
const std::string PointerLocalizerRobot::TOPIC_INPUT_DEPTH_IMAGE_DEFAULT = "/camera/depth_registered/image_raw";
const std::string PointerLocalizerRobot::TOPIC_RGB_CAMERA_INFO_DEFAULT = "/camera/rgb/camera_info";
const std::string PointerLocalizerRobot::TOPIC_POINTER_IMAGE = "/pointer_image";
const std::string PointerLocalizerRobot::TOPIC_POINTING_POINT = "/pointing_point";

PointerLocalizerRobot::PointerLocalizerRobot() :
		m_nodeHandle("~"), m_imageTransporter(m_nodeHandle), m_synchronizer(t_syncPolicy(10))
{
	m_nodeHandle.param(PARAM_INPUT_RGB_IMAGE_TOPIC, m_topicInputRGBImage, TOPIC_INPUT_RGB_IMAGE_DEFAULT);
	m_nodeHandle.param(PARAM_INPUT_DEPTH_IMAGE_TOPIC, m_topicInputDepthImage, TOPIC_INPUT_DEPTH_IMAGE_DEFAULT);
	m_nodeHandle.param(PARAM_RGB_CAMERA_INFO_TOPIC, m_topicRGBCameraInfo, TOPIC_RGB_CAMERA_INFO_DEFAULT);
	ROS_INFO("RGB image topic: %s", m_topicInputRGBImage.c_str());
	ROS_INFO("Depth image topic: %s", m_topicInputDepthImage.c_str());
	ROS_INFO("RGB camera info topic: %s", m_topicRGBCameraInfo.c_str());

	m_rgbImageSubscriber.subscribe(m_imageTransporter, m_topicInputRGBImage, 1);
	m_depthImageSubscriber.subscribe(m_imageTransporter, m_topicInputDepthImage, 1);
	m_rgbCameraInfoSubscriber = m_nodeHandle.subscribe(m_topicRGBCameraInfo, 1, &PointerLocalizerRobot::rgbCameraInfoReceivedCallback,
			this);

	m_synchronizer.connectInput(m_rgbImageSubscriber, m_depthImageSubscriber);
	m_synchronizer.registerCallback(boost::bind(&PointerLocalizerRobot::imagesReceivedCallback, this, _1, _2));
	m_imagePublisher = m_imageTransporter.advertise(TOPIC_POINTER_IMAGE, 1);
	//m_debugImagePublisher = m_imageTransporter.advertise("debug_image", 1);

	m_pointingPointPublisher = m_nodeHandle.advertise<geometry_msgs::PointStamped>(TOPIC_POINTING_POINT, 10);

	m_intrinsicRGBCameraMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
}

PointerLocalizerRobot::~PointerLocalizerRobot()
{

}

bool PointerLocalizerRobot::detectLaserPoint(const cv::Mat& f_rgbImage, cv::Point& f_laserpoint)
{
	const double MIN_CONTOUR_AREA = 60.0;
	const double MAX_CONTOUR_AREA = 250.0;
	const int MIN_V_VALUE = 220;
	bool l_found = false;
	cv::Mat l_grayImage;
	cv::Mat l_hsvImage;
	cv::cvtColor(f_rgbImage, l_grayImage, CV_BGR2GRAY);
	cv::cvtColor(f_rgbImage, l_hsvImage, CV_BGR2HSV);

	//Find areas that are brighter than their neighborhood
	cv::medianBlur(l_grayImage, l_grayImage, 3);
	cv::adaptiveThreshold(l_grayImage, l_grayImage, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 15, -15);
	cv::morphologyEx(l_grayImage, l_grayImage, CV_MOP_DILATE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

	//Find bright areas
	cv::Mat l_brightImage;
	cv::inRange(l_hsvImage, cv::Scalar(0, 0, MIN_V_VALUE), cv::Scalar(255, 255, 255), l_brightImage);
	cv::morphologyEx(l_brightImage, l_brightImage, CV_MOP_DILATE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15)));

	//Combine both areas
	cv::bitwise_and(l_grayImage, l_brightImage, l_grayImage);
	cv::morphologyEx(l_grayImage, l_grayImage, CV_MOP_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15)));

	/*sensor_msgs::ImagePtr l_imageMsg =
	 cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, l_grayImage).toImageMsg();
	 m_debugImagePublisher.publish(l_imageMsg);*/

	//Blob detection
	std::vector<std::vector<cv::Point> > l_blobs;
	std::vector<cv::Vec4i> l_hierarchy;
	cv::findContours(l_grayImage, l_blobs, l_hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	//Possible laser point found
	if (l_blobs.size() > 0)
	{
		int l_maxIndex = -1;
		unsigned char l_currentVValue;
		unsigned char l_maxVValue = 0;
		cv::Rect l_rect;
		cv::Point l_point;

		for (unsigned int i = 0; i < l_blobs.size(); i++)
		{
			//Check if blob is not too big
			l_rect = cv::boundingRect(l_blobs[i]);
			//std::cout << "Area: " << cv::contourArea(l_blobs[i]) << "\n";

			double l_area = cv::contourArea(l_blobs[i]);
			double l_perimeter = cv::arcLength(l_blobs[i], true);
			double l_circularity = (4 * M_PI * l_area) / pow(l_perimeter, 2);
			//std::cout << "circularity: " << l_circularity << "\n";

			if (l_area > MIN_CONTOUR_AREA && l_area < MAX_CONTOUR_AREA && l_circularity > 0.8 && l_circularity < 1.2)
			{
				//Find blob with highest v-value in hsv space (very bright pixel)
				l_point = l_rect.tl() + cv::Point(l_rect.width / 2, l_rect.height / 2);
				l_currentVValue = l_hsvImage.at<cv::Vec3b>(l_point)[2];
				if (l_currentVValue > l_maxVValue)
				{
					l_maxVValue = l_currentVValue;
					l_maxIndex = i;
				}
			}
		}

		//This is the laser point
		if (l_maxIndex >= 0 && l_maxVValue >= MIN_V_VALUE)
		{
			l_found = true;
			cv::Rect l_rect = cv::boundingRect(l_blobs[l_maxIndex]);
			f_laserpoint = l_rect.tl() + cv::Point(l_rect.width / 2, l_rect.height / 2);
		}
	}

	return l_found;
}

void PointerLocalizerRobot::imagesReceivedCallback(const sensor_msgs::ImageConstPtr& f_rgbImage,
		const sensor_msgs::ImageConstPtr& f_depthImage)
{
	cv::Mat l_rgbImage;
	image2Mat(*f_rgbImage, l_rgbImage, sensor_msgs::image_encodings::BGR8);

	//Find laser point
	cv::Point l_laserpoint;
	bool l_found = detectLaserPoint(l_rgbImage, l_laserpoint);

	//Laser point found
	if (l_found)
	{
		geometry_msgs::PointStamped l_pointerPose;
		cv::Mat l_depthImage;
		image2Mat(*f_depthImage, l_depthImage, f_depthImage->encoding);
		float l_scaleX = l_rgbImage.cols / l_depthImage.cols;
		float l_scaleY = l_rgbImage.rows / l_depthImage.rows;

		cv::circle(l_rgbImage, l_laserpoint, 10, cv::Scalar(0, 0, 255));

		//Calculate and publish pointer pose
		projectInto3D(l_depthImage, l_scaleX, l_scaleY, l_laserpoint, l_pointerPose);
		l_pointerPose.header.frame_id = f_rgbImage->header.frame_id;
		m_pointingPointPublisher.publish(l_pointerPose);
	}

	//Publish modified image
	sensor_msgs::ImagePtr l_imageMsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, l_rgbImage).toImageMsg();
	m_imagePublisher.publish(l_imageMsg);
}

void PointerLocalizerRobot::rgbCameraInfoReceivedCallback(const sensor_msgs::CameraInfo& f_cameraInfo)
{
	m_intrinsicRGBCameraMatrix.at<double>(0, 0) = f_cameraInfo.K.data()[0];
	m_intrinsicRGBCameraMatrix.at<double>(0, 1) = f_cameraInfo.K.data()[1];
	m_intrinsicRGBCameraMatrix.at<double>(0, 2) = f_cameraInfo.K.data()[2];
	m_intrinsicRGBCameraMatrix.at<double>(1, 0) = f_cameraInfo.K.data()[3];
	m_intrinsicRGBCameraMatrix.at<double>(1, 1) = f_cameraInfo.K.data()[4];
	m_intrinsicRGBCameraMatrix.at<double>(1, 2) = f_cameraInfo.K.data()[5];
	m_intrinsicRGBCameraMatrix.at<double>(2, 0) = f_cameraInfo.K.data()[6];
	m_intrinsicRGBCameraMatrix.at<double>(2, 1) = f_cameraInfo.K.data()[7];
	m_intrinsicRGBCameraMatrix.at<double>(2, 2) = f_cameraInfo.K.data()[8];
}

void PointerLocalizerRobot::projectInto3D(const cv::Mat& f_depthImage, float f_xScale, float f_yScale, const cv::Point& f_point2D,
		geometry_msgs::PointStamped& f_point3D) const
{
	const float MAX_DETECTION_DISTANCE = 4.0f;
	const float MIN_DETECTION_DISTANCE = 0.0f;

	//Calculate 3D- coordinates using depth image and intriniscs
	float x = f_depthImage.at<float>(f_point2D.y / f_yScale, f_point2D.x / f_xScale)
			* (static_cast<double>(f_point2D.x) - m_intrinsicRGBCameraMatrix.at<double>(0, 2))
			/ m_intrinsicRGBCameraMatrix.at<double>(0, 0);
	float y = f_depthImage.at<float>(f_point2D.y / f_yScale, f_point2D.x / f_xScale)
			* (static_cast<double>(f_point2D.y) - m_intrinsicRGBCameraMatrix.at<double>(1, 2))
			/ m_intrinsicRGBCameraMatrix.at<double>(1, 1);
	float z = f_depthImage.at<float>(f_point2D.y / f_yScale, f_point2D.x / f_xScale);

	//Check if the distance is plausible
	if (z >= MIN_DETECTION_DISTANCE && z < MAX_DETECTION_DISTANCE)
	{
		f_point3D.header.stamp = ros::Time::now();
		f_point3D.point.x = x;
		f_point3D.point.y = y;
		f_point3D.point.z = z;
	}
}
void PointerLocalizerRobot::image2Mat(const sensor_msgs::Image& f_image, cv::Mat& f_mat, const std::string& f_encoding) const
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(f_image, f_encoding);

	} catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	f_mat = cv::Mat(cv_ptr->image);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pointer_localizer_robot");
	PointerLocalizerRobot l_pointerLocalizer;
	ros::spin();
	return 0;
}
