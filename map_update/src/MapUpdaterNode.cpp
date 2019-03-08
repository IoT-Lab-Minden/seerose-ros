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
 * MapUpdaterNode.cpp
 *
 *  Created on: May 22, 2017
 *      Author: dsprute
 */

#include "map_update/MapUpdaterNode.h"

MapUpdaterNode::MapUpdaterNode() :
		m_nodeHandle("~")
{
	m_mapUpdateService = m_nodeHandle.advertiseService("update_map", &MapUpdaterNode::updateMap, this);
	m_mapSubscriber = m_nodeHandle.subscribe("/map", 1, &MapUpdaterNode::mapCallback, this);
	m_mapPublisher = m_nodeHandle.advertise<nav_msgs::OccupancyGrid>("/map", 10);
}

MapUpdaterNode::~MapUpdaterNode()
{

}

bool MapUpdaterNode::updateMap(lab_msgs::MapUpdate::Request& f_request, lab_msgs::MapUpdate::Response& f_response)
{
	ROS_INFO("Map update service called!");
	ROS_INFO("Number of border points: %d", static_cast<int>(f_request.border.polygon.points.size()));
	ROS_INFO("Border frame id: %s", f_request.border.header.frame_id.c_str());
	ROS_INFO("Border time: %d", f_request.border.header.stamp.sec);

	for (int i = 0; i < f_request.border.polygon.points.size(); i++)
	{
		ROS_INFO("x: %f, y: %f, z: %f", f_request.border.polygon.points.at(i).x, f_request.border.polygon.points.at(i).y,
				f_request.border.polygon.points.at(i).z);
	}

	ROS_INFO("Seed point frame id: %s", f_request.seedPoint.header.frame_id.c_str());
	ROS_INFO("Seed point time: %d", f_request.seedPoint.header.stamp.sec);
	ROS_INFO("Seed point x: %f, y: %f, z: %f", f_request.seedPoint.point.x, f_request.seedPoint.point.y, f_request.seedPoint.point.z);
	ROS_INFO("Semantic: %d", f_request.semantic);

	if (f_request.border.polygon.points.size() < 2)
	{
		ROS_INFO("Border has to be defined by at least two points!");
		f_response.message = "Border has to be defined by at least two points!";
		f_response.success = false;
		return true;
	}

	//Check if a prior map is available
	if (!m_priorMap.data.empty())
	{
		nav_msgs::OccupancyGrid l_borderMap;
		m_mapCreater.createBorderMap(m_priorMap, f_request.border, l_borderMap, f_request.seedPoint, (int) f_request.semantic);
		m_mapPublisher.publish(l_borderMap);

		ROS_INFO("Integrated borders into static map!");
		f_response.message = "Integrated borders into static map!";
		f_response.success = true;

	}
	else
	{
		ROS_INFO("No prior map could be loaded!");
		f_response.message = "No prior map could be loaded!";
		f_response.success = false;
	}

	return true;
}

void MapUpdaterNode::mapCallback(const nav_msgs::OccupancyGridConstPtr f_map)
{
	ROS_INFO("Map received!");
	m_priorMap = *f_map;
	ROS_INFO("Map width: %d, map height: %d", m_priorMap.info.width, m_priorMap.info.height);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_updater");
	MapUpdaterNode l_mapUpdater;
	ros::spin();
	return 0;
}

