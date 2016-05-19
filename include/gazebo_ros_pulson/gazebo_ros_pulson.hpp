#ifndef _GAZEBO_ROS_UWB_HPP_
#define _GAZEBO_ROS_UWB_HPP_

#include <string.h>

#include <yaml-cpp/yaml.h>

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#include <pulson_ros/RangeMeasurement.h>
#include <hector_gazebo_plugins/sensor_model.h>
#include <hector_gazebo_plugins/update_timer.h>

#include <gazebo/physics/physics.hh>

namespace gazebo
{
    class GazeboRosUwb : public ModelPlugin
    {
        public:
            // constructor
            GazeboRosUwb();

            // destructor
            virtual ~GazeboRosUwb();

        protected:

            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            virtual void Reset();
            virtual void Update();

            void ParseBeaconMapFile(std::string f);

        private:

            // the parent world
            physics::WorldPtr world_;

            // the linke referred to by this plugin
            physics::LinkPtr link_;

            ros::NodeHandle* node_handle_;
            ros::Publisher range_pub_;
            
            pulson_ros::RangeMeasurement range_;

            std::string namespace_;
            std::string link_name_;
            std::string range_topic_;
            std::string beacon_map_file_;

            UpdateTimer updateTimer_;
            event::ConnectionPtr updateConnection_;

    };
}

#endif
