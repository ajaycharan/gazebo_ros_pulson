#ifndef _GAZEBO_ROS_UWB_HPP_
#define _GAZEBO_ROS_UWB_HPP_

#include <string.h>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/package.h>

#include <pulson_ros/RangeMeasurement.h>
#include <hector_gazebo_plugins/sensor_model.h>
#include <hector_gazebo_plugins/update_timer.h>
#include <gazebo_ros_pulson/beacon.h>

#include <gazebo/physics/physics.hh>

namespace gazebo
{
    class GazeboRosPulson : public ModelPlugin
    {
        public:
            // constructor
            GazeboRosPulson();

            // destructor
            virtual ~GazeboRosPulson();

        protected:

            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            virtual void Reset();
            virtual void Update();

        private:

            // the parent world
            physics::WorldPtr world_;

            // the link referred to by this plugin
            physics::LinkPtr link_;

            ros::NodeHandle* nh_;
            ros::Publisher range_pub_;
            
            pulson_ros::RangeMeasurement range_;
            SensorModel range_error_model_;

            std::string namespace_;
            std::string link_name_;
            std::string range_topic_;

            int node_id_;
            std::vector<Beacon> beacons_;
            int num_beacons_;
            int counter_;

            UpdateTimer updateTimer_;
            event::ConnectionPtr updateConnection_;

            int ParseBeaconMapFile(std::string f);

    };
}

#endif
