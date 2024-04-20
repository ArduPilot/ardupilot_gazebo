#include "ardupilot_gazebo_sitl_viewer/SitlViewer.hpp"
#include <gz/msgs.hh>

SitlViewer::SitlViewer() : Node("SitlViewer") {

    rmw_qos_profile_t qos_sensor;
    qos_sensor.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    navsat_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/ap/navsat/navsat0",
        rclcpp::SensorDataQoS(),
        [this](std::shared_ptr<sensor_msgs::msg::NavSatFix> msg) {
            OnNavSatFix(msg);
        }
    );
}

void SitlViewer::OnNavSatFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Got NavSatFix from AP");

    gz::msgs::SphericalCoordinates req;
    req.set_elevation(msg->altitude);
    req.set_longitude_deg(msg->longitude);
    req.set_latitude_deg(msg->latitude);


    //! @todo Dynamically configure entity name based on frame_id from AP.
    //! @todo Spawn the entity if it doesn't exist.
    auto entity = req.mutable_entity();
    assert(entity != nullptr);
    // gz::msgs::Entity entity;
    entity->set_name("NavSat");
    entity->set_type(gz::msgs::Entity_Type_MODEL);
    // req.entity(entity);
    
    gz::msgs::Boolean rep;
    bool result;
    bool executed = gz_node_.Request("/world/map/set_spherical_coordinates", req, gz_service_timeout_ms_, rep, result);
    if (executed)
    { 
        if (result)
            RCLCPP_INFO_STREAM(this->get_logger(), "Response: [" << rep.data() << "]");
        else
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Service call timed out");
    }
}
