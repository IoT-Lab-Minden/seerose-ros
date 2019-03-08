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
 * MapUpdaterNode.h
 *
 *  Created on: May 22, 2017
 *      Author: dsprute
 */

#ifndef SRC_MAPUPDATERNODE_H_
#define SRC_MAPUPDATERNODE_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <lab_msgs/MapUpdate.h>

#include "map_update/MapCreater.hpp"

class MapUpdaterNode
{
public:
	MapUpdaterNode();
	virtual ~MapUpdaterNode();

private:
	/**
	 * Node handle.
	 */
	ros::NodeHandle m_nodeHandle;

	/**
	 * Service to provide a map update.
	 */
	ros::ServiceServer m_mapUpdateService;

	/**
	 * Subscriber for the prior map.
	 */
	ros::Subscriber m_mapSubscriber;

	/**
	 * Publisher for the updated map (topic: /map).
	 */
	ros::Publisher m_mapPublisher;

	/**
	 * Prior map.
	 */
	nav_msgs::OccupancyGrid m_priorMap;

	/**
	 * Object for updating maps.
	 */
	MapCreater m_mapCreater;

	/**
	 * Integrates the given map update request into the prior map and publishes the result.
	 * @param f_request The request containing an area to be integrated into the map.
	 * @param f_response the result.
	 * @return True on success.
	 */
	bool updateMap(lab_msgs::MapUpdate::Request& f_request, lab_msgs::MapUpdate::Response& f_response);

	/**
	 * Callback that is triggered whenever a new map is published.
	 *
	 * @param msg Pointer for the map data
	 */
	void mapCallback(const nav_msgs::OccupancyGridConstPtr f_map);
};

#endif /* SRC_MAPUPDATERNODE_H_ */
