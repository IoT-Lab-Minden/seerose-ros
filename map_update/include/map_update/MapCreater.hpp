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
 * MapCreater.h
 *
 *  Created on: 14.06.2016
 *      Author: dsprute
 */

#ifndef MAPCREATER_HPP_
#define MAPCREATER_HPP_

#include <ctime>
#include <limits>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>

#include <map_update/OGM_Values.hpp>

class MapCreater
{
public:
	MapCreater();
	virtual ~MapCreater();

	/**
	 * Integrates a border into the prior map.
	 *
	 * @param f_priorMap The prior map where borders need to be integrated.
	 * @param f_contour The contour/polygon that defines the border. Coordinates are in meters.
	 * @param f_borderMap The resulting border map containing the borders defined by the contour.
	 * @param f_seedPointFilling A point that defines the keep off area. It could be the inner area of a polygon
	 * 			or a site of a separating line. This area will be flood filled. Coordinates are in meters.
	 * @param f_fillId The id that is used to fill the keep off area. 100 --> occupied. Other ids can be further
	 * 			used to bring semantics into the map.
	 */
	void createBorderMap(const nav_msgs::OccupancyGrid& f_priorMap, geometry_msgs::PolygonStamped& f_contour,
			nav_msgs::OccupancyGrid& f_borderMap, geometry_msgs::PointStamped& f_seedPointFilling, int f_fillId = OCCUPIED) const;

private:

	/**
	 * TransformListener to convert between different coordinate frames.
	 */
	tf::TransformListener m_tflistener;

	/**
	 * Integrates a border polygon into the prior map.
	 *
	 * @param f_priorMap The prior map where borders need to be integrated.
	 * @param f_contour The polygon that defines the border. Coordinates are in pixels.
	 * @param f_borderMap The resulting border map containing the borders defined by the polygon.
	 * @param f_seedPointFilling A point that defines the keep off area. It is the inner or outer area of the polygon.
	 * 			Coordinates are in pixels.
	 * @param f_fillId The id that is used to fill the keep off area. 100 --> occupied. Other ids can be further
	 * 			used to bring semantics into the map.
	 */
	void createPolygonBorderMap(const nav_msgs::OccupancyGrid& f_priorMap, const std::vector<cv::Point>& f_contour,
			nav_msgs::OccupancyGrid& f_borderMap, const cv::Point f_seedPointFilling, int f_fillId = 100) const;

	/**
	 * Integrates a border line into the prior map. It separates the map into two areas.
	 *
	 * @param f_priorMap The prior map where borders need to be integrated.
	 * @param f_contour The line/curve that defines the borders. Coordinates are in pixels.
	 * @param f_borderMap The resulting border map containing the borders defined by the line/curve.
	 * @param f_seedPointFilling A point that defines the keep off area. It is one site of the separating line/curve.
	 * 			Coordinates are in pixels.
	 * @param f_fillId The id that is used to fill the keep off area. 100 --> occupied. Other ids can be further
	 * 			used to bring semantics into the map.
	 */
	void createLineBorderMap(const nav_msgs::OccupancyGrid& f_priorMap, const std::vector<cv::Point>& f_contour,
			nav_msgs::OccupancyGrid& f_borderMap, const cv::Point f_seedPointFilling, int f_fillId = 100) const;

	/**
	 * Returns true if the given contour is a polygon. The contour is considered to be a
	 * polygon if its first and last point are close to each other (<1 meter distance) and if the polygon exceeds a
	 * certain length (0.3 meters).
	 *
	 * @param f_contour The contour to be checked. Coordinates are in meters.
	 * @return True if the given contour is a polygon.
	 */
	bool isPolygon(const geometry_msgs::Polygon& f_contour) const;

	/**
	 * Calculates the slope between the given points.
	 *
	 * @param f_point1 The first point.
	 * @param f_point2 The second point.
	 * @return The slope between the points.
	 */
	double getSlope(const cv::Point f_point1, const cv::Point f_point2) const;

	/**
	 * Draws an extended contour/line/curve in both directions. The directions of the extensions are determined
	 * by the beginning and ending two contour points.
	 *
	 * @param f_image The image containing the contour.
	 * @param f_contour The contour to be extended.
	 * @param f_color The color of the extensions.
	 */
	void drawExtendedLine(cv::Mat& f_image, const std::vector<cv::Point>& f_contour, const cv::Scalar f_color) const;

	/**
	 * Draws a half line through the given points. The direction of the line depends on the order of the
	 * given points. If f_point2.x > f_point1.x, the line is extended to the left, otherwise to the right.
	 * If f_point2.x == f_point1.x and f_point2.y > f_point1.y, the line is extended upwards, otherwise downwards.
	 *
	 * @param f_image The image that will be drawn.
	 * @param f_point1 The first point.
	 * @param f_point2 The second point.
	 * @param f_color The color of the line to be drawn.
	 */
	void drawHalfLine(cv::Mat& f_image, const cv::Point f_point1, const cv::Point f_point2, const cv::Scalar f_color) const;

	/**
	 * Draws a full line through the given points.
	 *
	 * @param f_image The image that will be drawn.
	 * @param f_point1 The first point.
	 * @param f_point2 The second point.
	 * @param f_color The color of the line to be drawn.
	 */
	void drawFullLine(cv::Mat& f_image, const cv::Point f_point1, const cv::Point f_point2, const cv::Scalar f_color) const;

	/**
	 * Draws the given contour (no polygon as in OpenCV!).
	 *
	 * @param f_image The image containing the contour.
	 * @param f_contour The contour to be drawn.
	 * @param f_color The color of the contour.
	 */
	void drawLineSegments(cv::Mat& f_image, const std::vector<cv::Point>& f_contour, const cv::Scalar f_color) const;

	/**
	 * Converts an occupancy grid map to an image. A pixel value represents the occupancy probability in the range
	 * of [0; 100]. A -1 represents an unknown cell.
	 *
	 * @param f_map The occupancy grid map that will be visualized.
	 * @param f_mapImage The image of the map with probabilities in the range 0 and 100. Unknown cells are marked with a -1.
	 * @param f_flip If true, the resulting image is flipped vertically. This can be useful because the origin of an image
	 * 			is in the top left, while the origin of a map is in the bottom left.
	 */
	void mapToImage(const nav_msgs::OccupancyGrid& f_map, cv::Mat& f_mapImage, bool f_flip = false) const;

	/**
	 * Converts a map image with occupancy probabilities in the range of 0 and 100 to an occupancy grid map structure.
	 *
	 * @param f_mapImage The image of the map with probabilities in the range 0 and 100.
	 * @param f_map The resulting occupancy grid map.
	 * @param f_flip If true, the image is flipped vertically before conversion. This can be useful because the origin of an image
	 * 			is in the top left, while the origin of a map is in the bottom left.
	 */
	void imageToMap(const cv::Mat& f_mapImage, nav_msgs::OccupancyGrid& f_map, bool f_flip = false) const;

	/**
	 * Transforms a polygon into the given target frame.
	 * @param f_points The polygon points.
	 * @param f_targetFrameId The target frame id.
	 */
	void transformPolygon(geometry_msgs::PolygonStamped& f_points, const std::string f_targetFrameId) const;

	/**
	 * Transforms a point into the given target frame.
	 * @param f_point The point.
	 * @param f_targetFrameId The target frame id.
	 */
	void transformPoint(geometry_msgs::PointStamped& f_point, const std::string f_targetFrameId) const;

	/**
	 * Returns the length of the polygon.
	 * @param f_polygon The polygon.
	 * @return The length of the polygon.
	 */
	float getLength(const geometry_msgs::Polygon& f_polygon) const;
};

#endif /* MAPCREATER_HPP_ */

