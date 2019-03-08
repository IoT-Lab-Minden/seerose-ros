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
 * PointerLocalizerIE.cpp
 *
 *  	Created on: 03.01.2018
 *      Author: dsprute
 */

#include "pointer_detection/PointerLocalizerIE.h"

const std::string PointerLocalizerIE::TF_MAP = "/map";
const std::string PointerLocalizerIE::PARAM_INPUT_RGB_IMAGE_TOPIC = "input_rgb_topic";
const std::string PointerLocalizerIE::PARAM_RGB_CAMERA_INFO_TOPIC = "rgb_camera_info_topic";
const std::string PointerLocalizerIE::TOPIC_INPUT_RGB_IMAGE_DEFAULT = "/camera/rgb/image_raw";
const std::string PointerLocalizerIE::TOPIC_RGB_CAMERA_INFO_DEFAULT = "/camera/rgb/camera_info";
const std::string PointerLocalizerIE::TOPIC_POINTER_IMAGE = "/pointer_image";
const std::string PointerLocalizerIE::TOPIC_POINTING_POINT = "/pointing_point";

PointerLocalizerIE::PointerLocalizerIE() :
		m_nodeHandle("~"), m_imageTransporter(m_nodeHandle)
{
	m_nodeHandle.param(PARAM_INPUT_RGB_IMAGE_TOPIC, m_topicInputRGBImage, TOPIC_INPUT_RGB_IMAGE_DEFAULT);
	m_nodeHandle.param(PARAM_RGB_CAMERA_INFO_TOPIC, m_topicRGBCameraInfo, TOPIC_RGB_CAMERA_INFO_DEFAULT);
	ROS_INFO("RGB image topic: %s", m_topicInputRGBImage.c_str());
	ROS_INFO("RGB camera info topic: %s", m_topicRGBCameraInfo.c_str());

	m_imageSubscriber = m_imageTransporter.subscribe(m_topicInputRGBImage, 1, &PointerLocalizerIE::imageReceivedCallback, this);
	m_rgbCameraInfoSubscriber = m_nodeHandle.subscribe(m_topicRGBCameraInfo, 1, &PointerLocalizerIE::rgbCameraInfoReceivedCallback, this);
	m_imagePublisher = m_imageTransporter.advertise(TOPIC_POINTER_IMAGE, 1);
	//m_debugImagePublisher = m_imageTransporter.advertise("debug_image", 1);
	m_pointingPointPublisher = m_nodeHandle.advertise<geometry_msgs::PointStamped>(TOPIC_POINTING_POINT, 10);

	m_intrinsicRGBCameraMatrix = cv::Mat::zeros(3, 3, CV_64FC1);

	//Initialize ground plane
	m_groundPlane.header.frame_id = TF_MAP;
	m_groundPlane.normalVector.x = 0.0;
	m_groundPlane.normalVector.y = 0.0;
	m_groundPlane.normalVector.z = 1.0;
	m_groundPlane.supportVector.x = 0.0;
	m_groundPlane.supportVector.y = 0.0;
	m_groundPlane.supportVector.z = 0.0;
}

PointerLocalizerIE::~PointerLocalizerIE()
{

}

bool PointerLocalizerIE::detectLaserPoint(const cv::Mat& f_rgbImage, cv::Point& f_laserpoint)
{
	const double MIN_CONTOUR_AREA = 80.0;
	const double MAX_CONTOUR_AREA = 230.0;
	const int MIN_V_VALUE = 180;
	bool l_found = false;
	cv::Mat l_grayImage;
	cv::Mat l_hsvImage;
	cv::cvtColor(f_rgbImage, l_grayImage, CV_BGR2GRAY);
	cv::cvtColor(f_rgbImage, l_hsvImage, CV_BGR2HSV);

	//Find areas that are brighter than their neighborhood
	cv::medianBlur(l_grayImage, l_grayImage, 3);
	cv::adaptiveThreshold(l_grayImage, l_grayImage, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 15, -15);
	cv::morphologyEx(l_grayImage, l_grayImage, CV_MOP_DILATE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

	//Find foreground
	cv::Mat l_foreground;
	m_backgroundSubtractor(f_rgbImage, l_foreground);
	cv::morphologyEx(l_foreground, l_foreground, CV_MOP_DILATE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
	cv::threshold(l_foreground, l_foreground, 254, 255, CV_THRESH_BINARY);

	//Combine areas
	cv::bitwise_and(l_grayImage, l_foreground, l_grayImage);
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
			double l_area = cv::contourArea(l_blobs[i]);
			double l_perimeter = cv::arcLength(l_blobs[i], true);
			double l_circularity = (4 * M_PI * l_area) / pow(l_perimeter, 2);

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

void PointerLocalizerIE::imageReceivedCallback(const sensor_msgs::ImageConstPtr& f_image)
{
	m_lastRGBImage = *f_image;
	cv::Mat l_rgbImage;
	image2Mat(*f_image, l_rgbImage, sensor_msgs::image_encodings::BGR8);

	//Find laser point
	cv::Point l_laserpoint;
	bool l_found = detectLaserPoint(l_rgbImage, l_laserpoint);

	//Laser point found
	if (l_found)
	{
		geometry_msgs::PoseStamped l_pointerPose;

		//Draw laserpoint
		geometry_msgs::PointStamped l_pointSpace;
		if (getPositionOnPlane(l_laserpoint, m_groundPlane, l_pointSpace))
		{
			m_pointingPointPublisher.publish(l_pointSpace);
		}
		m_laserpoints.push_back(l_laserpoint);
	}

	drawLaserpoints(l_rgbImage, m_laserpoints, cv::Scalar(0, 0, 255));

	//Publish modified image
	sensor_msgs::ImagePtr l_imageMsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, l_rgbImage).toImageMsg();
	m_imagePublisher.publish(l_imageMsg);
}

void PointerLocalizerIE::rgbCameraInfoReceivedCallback(const sensor_msgs::CameraInfo& f_cameraInfo)
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

void PointerLocalizerIE::drawLaserpoints(cv::Mat& f_rgbImage, const std::vector<cv::Point>& f_laserpoints, cv::Scalar f_color) const
{
	const int CIRCLE_RADIUS = 10;
	for (unsigned int i = 0; i < f_laserpoints.size(); i++)
	{
		cv::circle(f_rgbImage, f_laserpoints[i], CIRCLE_RADIUS, f_color);
	}
}

void PointerLocalizerIE::image2Mat(const sensor_msgs::Image& f_image, cv::Mat& f_mat, const std::string& f_encoding) const
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

bool PointerLocalizerIE::getPositionOnPlane(const cv::Point2f& f_imagePoint, const lab_msgs::Plane& f_plane,
		geometry_msgs::PointStamped& f_spacePoint) const
{
	bool l_returnValue = false;
	if (!m_intrinsicRGBCameraMatrix.empty())
	{
		lab_msgs::Line l_pixelRay;
		l_pixelRay.header.frame_id = m_lastRGBImage.header.frame_id;
		l_pixelRay.header.stamp = ros::Time::now();
		l_pixelRay.directionVector.x = (f_imagePoint.x - m_intrinsicRGBCameraMatrix.at<double>(0, 2))
				/ m_intrinsicRGBCameraMatrix.at<double>(0, 0);
		l_pixelRay.directionVector.y = (f_imagePoint.y - m_intrinsicRGBCameraMatrix.at<double>(1, 2))
				/ m_intrinsicRGBCameraMatrix.at<double>(1, 1);
		l_pixelRay.directionVector.z = 1.0;

		geometry_msgs::PointStamped l_intersection;
		if (getPlaneIntersection(f_plane, l_pixelRay, f_spacePoint))
		{
			l_returnValue = true;
		}
	}
	return l_returnValue;
}

bool PointerLocalizerIE::getPlaneIntersection(const lab_msgs::Plane& f_plane, const lab_msgs::Line& f_line,
		geometry_msgs::PointStamped& f_intersectionPoint) const
{
	bool l_returnValue = false;

	try
	{
		//Transform line's direction vector into plane's coordinate frame
		geometry_msgs::Vector3Stamped l_lineDirectionVector;
		tf::Vector3 l_tfLineDirectionvector;
		l_lineDirectionVector.header = f_line.header;
		l_lineDirectionVector.vector = f_line.directionVector;
		m_tflistener.transformVector(f_plane.header.frame_id, l_lineDirectionVector, l_lineDirectionVector);
		tf::vector3MsgToTF(l_lineDirectionVector.vector, l_tfLineDirectionvector);

		//Transform line's support vector into plane's coordinate frame
		geometry_msgs::PointStamped l_lineSupportVector;
		l_lineSupportVector.header = f_line.header;
		l_lineSupportVector.point = f_line.supportVector;
		m_tflistener.transformPoint(f_plane.header.frame_id, l_lineSupportVector, l_lineSupportVector);
		tf::Vector3 l_tfLineSupportVector(l_lineSupportVector.point.x, l_lineSupportVector.point.y, l_lineSupportVector.point.z);

		//Convert plane's normal and support vector
		tf::Vector3 l_tfPlaneNormalvector;
		tf::vector3MsgToTF(f_plane.normalVector, l_tfPlaneNormalvector);
		tf::Vector3 l_tfPlaneSupportVector(f_plane.supportVector.x, f_plane.supportVector.y, f_plane.supportVector.z);

		//Calculate d, i.e. the scale factor for the line's direction vector
		float d = ((l_tfPlaneSupportVector - l_tfLineSupportVector).dot(l_tfPlaneNormalvector))
				/ (l_tfLineDirectionvector.dot(l_tfPlaneNormalvector));

		//Calculate the intersection point between line and plane
		tf::Vector3 l_intersection = l_tfLineSupportVector + d * l_tfLineDirectionvector;
		geometry_msgs::Vector3 l_tempVector;
		tf::vector3TFToMsg(l_intersection, l_tempVector);

		f_intersectionPoint.header.frame_id = f_plane.header.frame_id;
		f_intersectionPoint.header.stamp = ros::Time::now();
		f_intersectionPoint.point.x = l_tempVector.x;
		f_intersectionPoint.point.y = l_tempVector.y;
		f_intersectionPoint.point.z = l_tempVector.z;
		l_returnValue = true;

	} catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s", ex.what());
		return false;
	}

	return l_returnValue;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pointer_localizer_ie");
	PointerLocalizerIE l_pointerLocalizer;
	ros::spin();
	return 0;
}
