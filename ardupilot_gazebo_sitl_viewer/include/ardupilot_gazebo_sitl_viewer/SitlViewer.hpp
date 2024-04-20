/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <string>
#include <memory>
#include <gz/transport.hh>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

//! @class This class creates a gazebo world on the fly to view SITL.
//!        It interfaces with the DDS interface of Ardupilot.
//!        It relies on the ArduPilot SITL FDM for the aircraft dynamics.
class SitlViewer: public rclcpp::Node
 {
    public:
        SitlViewer();
    private:

        void OnNavSatFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
        gz::transport::Node gz_node_;
        const unsigned int gz_service_timeout_ms_ {5000};
        std::string set_spherical_coords_topic {"/world/map/set_spherical_coordinates"};

};