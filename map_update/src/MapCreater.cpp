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
 * MapCreater.cpp
 *
 *  Created on: 14.06.2016
 *      Author: dsprute
 */

#include "map_update/MapCreater.hpp"

MapCreater::MapCreater()
{
	ros::Time::init();
}

MapCreater::~MapCreater()
{

}

void MapCreater::createBorderMap(const nav_msgs::OccupancyGrid& f_priorMap, geometry_msgs::PolygonStamped& f_contour,
		nav_msgs::OccupancyGrid& f_borderMap, geometry_msgs::PointStamped& f_seedPointFilling, int f_fillId) const
{
	ROS_INFO("createBorderMap(): call");

	if (f_contour.polygon.points.size() < 2)
	{
		ROS_INFO("Not enough points to integrate border!");
		return;
	}

	//Transform points into map coordinate frame if necessary
	if (f_priorMap.header.frame_id != f_contour.header.frame_id)
	{
		transformPolygon(f_contour, f_priorMap.header.frame_id);
	}
	if (f_priorMap.header.frame_id != f_seedPointFilling.header.frame_id)
	{
		transformPoint(f_seedPointFilling, f_priorMap.header.frame_id);
		ROS_INFO("Transformed seed point x: %f, y: %f, z: %f", f_seedPointFilling.point.x, f_seedPointFilling.point.y,
				f_seedPointFilling.point.z);
	}

	//Transform f_contour and f_seedPointFilling into image coordinates (pixel) according to the map resolution
	std::vector<cv::Point> l_contour(f_contour.polygon.points.size());
	int x, y;
	for (unsigned int i = 0; i < f_contour.polygon.points.size(); i++)
	{
		x = static_cast<int>((f_contour.polygon.points[i].x - f_priorMap.info.origin.position.x) * (1.0f / f_priorMap.info.resolution));
		y = static_cast<int>((f_contour.polygon.points[i].y - f_priorMap.info.origin.position.y) * (1.0f / f_priorMap.info.resolution));
		l_contour[i] = cv::Point(x, y);
	}
	int l_xPos = static_cast<int>((f_seedPointFilling.point.x - f_priorMap.info.origin.position.x) * (1.0f / f_priorMap.info.resolution));
	int l_yPos = static_cast<int>((f_seedPointFilling.point.y - f_priorMap.info.origin.position.y) * (1.0f / f_priorMap.info.resolution));
	cv::Point l_seedFillingPoint(l_xPos, l_yPos);

	if (isPolygon(f_contour.polygon))
	{
		//Border is a polygon
		ROS_INFO("Border is a polygon!");
		createPolygonBorderMap(f_priorMap, l_contour, f_borderMap, l_seedFillingPoint, f_fillId);
	}
	else
	{
		//Border is a line/curve
		ROS_INFO("Border is a line!");
		createLineBorderMap(f_priorMap, l_contour, f_borderMap, l_seedFillingPoint, f_fillId);
	}

	//For evaluation purposes
	/*time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, sizeof(buffer), "%d-%m-%Y-%I:%M:%S", timeinfo);
	std::string str(buffer);

	cv::Mat l_saveImage;
	mapToImage(f_borderMap, l_saveImage, true);
	l_saveImage += 5;
	cv::imwrite("/home/ubuntu/" + str + "test.pgm", l_saveImage);*/

}

void MapCreater::createPolygonBorderMap(const nav_msgs::OccupancyGrid& f_priorMap, const std::vector<cv::Point>& f_contour,
		nav_msgs::OccupancyGrid& f_borderMap, const cv::Point f_seedPointFilling, int f_fillId) const
{
	//Check if the given width and height of the map is enough for the polygon
	cv::Rect l_boundingRect = cv::boundingRect(f_contour);
	int l_mapCols = f_priorMap.info.width;
	int l_mapRows = f_priorMap.info.height;
	if (l_boundingRect.br().x > static_cast<int>(f_priorMap.info.width))
	{
		l_mapCols = l_boundingRect.br().x;
	}
	if (l_boundingRect.br().y > l_mapRows)
	{
		l_mapRows = l_boundingRect.br().y;
	}

	cv::Mat l_mapImage;
	std::vector<std::vector<cv::Point> > l_contours;
	l_contours.push_back(f_contour);

	//Fill inner of the polygon as occupied
	if (cv::pointPolygonTest(f_contour, f_seedPointFilling, false) > 0)
	{
		ROS_INFO("Fill!");
		l_mapImage = cv::Mat(l_mapRows, l_mapCols, CV_8SC1, cv::Scalar(UNKNOWN));

		//Copy prior map into map image. The prior image can be a subimage of the map image.
		cv::Mat l_priorMap;
		mapToImage(f_priorMap, l_priorMap);
		l_priorMap.copyTo(l_mapImage(cv::Rect(0, 0, l_priorMap.cols, l_priorMap.rows)));

		//Draw borders and fill the inner area of the borders with f_fillId
		for (unsigned int i = 0; i < l_contours.size(); i++)
		{
			drawContours(l_mapImage, l_contours, i, cv::Scalar(f_fillId), -1);
		}
	}
	//Fill inner of the polygon as free
	else
	{
		ROS_INFO("No fill!");
		l_mapImage = cv::Mat(l_mapRows, l_mapCols, CV_8SC1, cv::Scalar(f_fillId));

		//Draw borders and fill the inner area of the borders as free
		for (unsigned int i = 0; i < l_contours.size(); i++)
		{
			drawContours(l_mapImage, l_contours, i, cv::Scalar(FREE), -1);
		}
	}
	//Create message for overlayed map
	imageToMap(l_mapImage, f_borderMap);
	f_borderMap.header.stamp = ros::Time::now();
	f_borderMap.header.frame_id = f_priorMap.header.frame_id;
	f_borderMap.info.map_load_time = ros::Time::now();
	f_borderMap.info.resolution = f_priorMap.info.resolution;
	f_borderMap.info.width = l_mapCols;
	f_borderMap.info.height = l_mapRows;
	f_borderMap.info.origin = f_priorMap.info.origin;
}

void MapCreater::createLineBorderMap(const nav_msgs::OccupancyGrid& f_priorMap, const std::vector<cv::Point>& f_contour,
		nav_msgs::OccupancyGrid& f_borderMap, const cv::Point f_seedPointFilling, int f_fillId) const
{
	//Check if the given width and height of the map is enough for the polygon
	cv::Rect l_boundingRect = cv::boundingRect(f_contour);
	int l_mapCols = f_priorMap.info.width;
	int l_mapRows = f_priorMap.info.height;
	if (l_boundingRect.br().x > static_cast<int>(f_priorMap.info.width))
	{
		l_mapCols = l_boundingRect.br().x;
	}
	if (l_boundingRect.br().y > l_mapRows)
	{
		l_mapRows = l_boundingRect.br().y;
	}

	cv::Mat l_mapImage = cv::Mat(l_mapRows, l_mapCols, CV_8SC1, cv::Scalar(-1));

	//Copy prior map into map image. The prior image can be a subimage of the map image.
	cv::Mat l_priorMap;
	mapToImage(f_priorMap, l_priorMap);
	l_priorMap.copyTo(l_mapImage(cv::Rect(0, 0, l_priorMap.cols, l_priorMap.rows)));

	//Create a new binary image for blob detection because blob detection changes the input image
	cv::Mat l_imageCopy = l_mapImage.clone();
	l_imageCopy.convertTo(l_imageCopy, CV_8U);

	//Draws the contour and their extensions.
	drawLineSegments(l_imageCopy, f_contour, cv::Scalar(f_fillId));
	drawExtendedLine(l_imageCopy, f_contour, cv::Scalar(f_fillId));
	cv::threshold(l_imageCopy, l_imageCopy, 10, f_fillId, CV_THRESH_BINARY_INV);
	cv::morphologyEx(l_imageCopy, l_imageCopy, CV_MOP_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

	std::vector<std::vector<cv::Point> > l_areas;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(l_imageCopy, l_areas, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	//Occupy inner areas of contours that contain f_seedPointFilling
	for (unsigned int i = 0; i < l_areas.size(); i++)
	{
		if (cv::pointPolygonTest(l_areas[i], f_seedPointFilling, false) > 0)
		{
			drawContours(l_mapImage, l_areas, i, cv::Scalar(f_fillId), -1);
		}
	}
	ROS_INFO("Seed point x: %d", f_seedPointFilling.x);
	ROS_INFO("Seed point y: %d", f_seedPointFilling.y);

	//Create message for overlayed map
	imageToMap(l_mapImage, f_borderMap);
	f_borderMap.header.stamp = ros::Time::now();
	f_borderMap.header.frame_id = f_priorMap.header.frame_id;
	f_borderMap.info.map_load_time = ros::Time::now();
	f_borderMap.info.resolution = f_priorMap.info.resolution;
	f_borderMap.info.width = l_mapCols;
	f_borderMap.info.height = l_mapRows;
	f_borderMap.info.origin = f_priorMap.info.origin;
}

bool MapCreater::isPolygon(const geometry_msgs::Polygon& f_contour) const
{
	const double DISTANCE_THRESHOLD_IN_METERS = 1.0;
	const double LENGTH_THRESHOLD_IN_METERS = 0.3;
	bool l_result = false;

	if (f_contour.points.size() > 2)
	{
		cv::Point2f l_firstPoint(f_contour.points[0].x, f_contour.points[0].y);
		cv::Point2f l_lastPoint(f_contour.points[f_contour.points.size() - 1].x, f_contour.points[f_contour.points.size() - 1].y);
		double l_distance = cv::norm(l_firstPoint - l_lastPoint);
		ROS_INFO("Distance: %f", l_distance);
		double l_length = getLength(f_contour);
		ROS_INFO("Length: %f", l_length);

		if (l_length > LENGTH_THRESHOLD_IN_METERS && l_distance < DISTANCE_THRESHOLD_IN_METERS)
		{
			l_result = true;
		}
	}
	return l_result;
}

double MapCreater::getSlope(const cv::Point f_point1, const cv::Point f_point2) const
{
	double l_slope = std::numeric_limits<double>::max();
	int l_xDiff = f_point2.x - f_point1.x;
	if (l_xDiff != 0)
	{
		l_slope = (double) (f_point2.y - f_point1.y) / l_xDiff;
	}
	return l_slope;
}

void MapCreater::drawExtendedLine(cv::Mat& f_image, const std::vector<cv::Point>& f_contour, const cv::Scalar f_color) const
{
	ROS_INFO("Draw extended line!");
	if (f_contour.size() == 2)
	{
		drawFullLine(f_image, f_contour[0], f_contour[f_contour.size() - 1], f_color);
	}
	else if (f_contour.size() > 2)
	{
		drawHalfLine(f_image, f_contour[0], f_contour[1], f_color);
		drawHalfLine(f_image, f_contour[f_contour.size() - 1], f_contour[f_contour.size() - 2], f_color);
	}
}

void MapCreater::drawHalfLine(cv::Mat& f_image, const cv::Point f_point1, const cv::Point f_point2, const cv::Scalar f_color) const
{
	const int MIN_OBSTACLE_DIMENSION_IN_PIXELS = 3;
	double l_slope = getSlope(f_point1, f_point2);
	cv::Point l_diff = f_point2 - f_point1;
	cv::Point l_orientationPoint;
	cv::Point l_extendPoint(f_point1);

	if (l_diff.x > 0)
	{
		ROS_INFO("Extend left!");
		l_orientationPoint.x = 0;
		l_orientationPoint.y = static_cast<int>((l_orientationPoint.x - f_point1.x) * l_slope + f_point1.y);
	}
	else if (l_diff.x < 0)
	{
		ROS_INFO("Extend right!");
		l_orientationPoint.x = f_image.cols;
		l_orientationPoint.y = static_cast<int>((l_orientationPoint.x - f_point1.x) * l_slope + f_point1.y);
	}
	else
	{
		//Extend up
		if (l_diff.y > 0)
		{
			ROS_INFO("Extend upward!");
			l_orientationPoint.x = f_point1.x;
			l_orientationPoint.y = 0;
		}
		else
		{
			ROS_INFO("Extend downward!");
			l_orientationPoint.x = f_point1.x;
			l_orientationPoint.y = f_image.rows;
		}
	}

	int l_counter = 0;
	cv::LineIterator it(f_image, f_point1, l_orientationPoint, 8);
	for (int i = 0; i < it.count; i++, ++it)
	{
		l_extendPoint = it.pos();
		if (f_image.at<signed char>(l_extendPoint) == OCCUPIED)
		{
			l_counter++;
			if (l_counter >= MIN_OBSTACLE_DIMENSION_IN_PIXELS)
			{
				break;
			}
		}
		else
		{
			l_counter = 0;
		}
	}
	line(f_image, l_extendPoint, f_point1, f_color, 2);
}

void MapCreater::drawFullLine(cv::Mat& f_image, const cv::Point f_point1, const cv::Point f_point2, const cv::Scalar f_color) const
{
	double l_slope = getSlope(f_point1, f_point2);
	cv::Point l_leftPoint(0, 0);
	cv::Point l_rightPoint(f_image.cols, f_image.rows);

	//Normal line function y =  (x-x_1)*slope+y_1
	if (f_point1.x != f_point2.x)
	{
		l_leftPoint.y = static_cast<int>((l_leftPoint.x - f_point1.x) * l_slope + f_point1.y);
		l_rightPoint.y = static_cast<int>((l_rightPoint.x - f_point2.x) * l_slope + f_point2.y);
	}
	//Vertical line
	else
	{
		l_leftPoint.x = f_point1.x;
		l_rightPoint.x = f_point2.x;
	}
	line(f_image, l_leftPoint, l_rightPoint, f_color, 2);
}

void MapCreater::drawLineSegments(cv::Mat& f_image, const std::vector<cv::Point>& f_contour, const cv::Scalar f_color) const
{
	ROS_INFO("Draw line segments!");
	for (unsigned int i = 0; i < f_contour.size() - 1; i++)
	{
		line(f_image, f_contour[i], f_contour[i + 1], f_color, 2);
	}
}

void MapCreater::imageToMap(const cv::Mat& f_mapImage, nav_msgs::OccupancyGrid& f_map, bool f_flip) const
{
	cv::Mat l_flippedImage = f_mapImage;

	if (f_flip)
	{
		cv::flip(f_mapImage, l_flippedImage, 0);
	}

	std::vector<signed char> l_data(l_flippedImage.total());
	for (int i = 0; i < l_flippedImage.rows; i++)
	{
		for (int j = 0; j < l_flippedImage.cols; j++)
		{
			l_data[i * l_flippedImage.cols + j] = l_flippedImage.at<signed char>(i, j);
		}
	}
	//Create the occupancy grid message
	f_map.header.stamp = ros::Time::now();
	f_map.info.width = l_flippedImage.cols;
	f_map.info.height = l_flippedImage.rows;
	f_map.data = l_data;
}

void MapCreater::mapToImage(const nav_msgs::OccupancyGrid& f_map, cv::Mat& f_mapImage, bool f_flip) const
{
	int l_size = f_map.info.width * f_map.info.height;
	f_mapImage = cv::Mat(f_map.info.height, f_map.info.width, CV_8SC1);
	for (int i = 0; i < l_size; i++)
	{
		f_mapImage.data[i] = static_cast<signed char>(f_map.data[i]);
	}
	if (f_flip)
	{
		cv::flip(f_mapImage, f_mapImage, 0);
	}
}

void MapCreater::transformPolygon(geometry_msgs::PolygonStamped& f_points, const std::string f_targetFrameId) const
{
	if (f_points.header.frame_id != f_targetFrameId)
	{
		geometry_msgs::PointStamped l_tempPoint;
		for (int i = 0; i < f_points.polygon.points.size(); i++)
		{
			l_tempPoint.header = f_points.header;
			l_tempPoint.point.x = f_points.polygon.points[i].x;
			l_tempPoint.point.y = f_points.polygon.points[i].y;
			l_tempPoint.point.z = f_points.polygon.points[i].z;
			transformPoint(l_tempPoint, f_targetFrameId);

			f_points.polygon.points[i].x = l_tempPoint.point.x;
			f_points.polygon.points[i].y = l_tempPoint.point.y;
			f_points.polygon.points[i].z = l_tempPoint.point.z;

			ROS_INFO("Transformed point x: %f, y: %f, z: %f", f_points.polygon.points[i].x, f_points.polygon.points[i].y,
					f_points.polygon.points[i].z);

		}
		f_points.header = l_tempPoint.header;
	}
}

void MapCreater::transformPoint(geometry_msgs::PointStamped& f_point, const std::string f_targetFrameId) const
{
	try
	{
		m_tflistener.transformPoint(f_targetFrameId, f_point, f_point);

	} catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s", ex.what());
		return;
	}
}

float MapCreater::getLength(const geometry_msgs::Polygon& f_polygon) const
{
	double l_sum = 0.0;
	for (int i = 0; i < f_polygon.points.size() - 1; i++)
	{
		cv::Point2f l_point1(f_polygon.points[i].x, f_polygon.points[i].y);
		cv::Point2f l_point2(f_polygon.points[i + 1].x, f_polygon.points[i + 1].y);
		l_sum += cv::norm(l_point1 - l_point2);
	}
	return l_sum;
}
