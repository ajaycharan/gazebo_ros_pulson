#include <gazebo_ros_pulson/gazebo_ros_pulson.hpp>

namespace gazebo
{

    GazeboRosUwb::GazeboRosUwb(){}

    GazeboRosUwb::~GazeboRosUwb(){}

    void GazeboRosUwb::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

        world_ = _model->GetWorld();

        //////////////////////// Load Params /////////////////////////////

        if (!_sdf->HasElement("robotNamespace"))
        {
            namespace_.clear();
        }
        else
        {
            namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
        }

        if (!_sdf->HasElement("bodyName"))
        {
            link_ = _model->GetLink();
            link_name_ = link_->GetName();
        }
        else
        {
            link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
            link_ = _model->GetLink(link_name_);
        }

        if (!link)
        {
            ROS_FATAL("GazeboRosUwb plugin error bodyName: %s does not exist\n", link_name_.c_str());
            return;
        }

        ////////////////////////// defaults ///////////////////////////////

        if (_sdf->HasElement("topicName"))
        {
            range_topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();
        }
        else
        {
            range_topic_ = "ranges";
        }

        if (_sdf->HasElement("beaconMapFile"))
        {
            beacon_map_file_ = _sdf->GetElement("beaconMapFile")->GetValue()->GetAsString();
        }
        else
        {
            ROS_FATAL("GazeboRosUwb plugin requires a beaconMapFile\n");
            return;
        }

    }

    void GazeboRosUwb::Reset(){}

    void GazeboRosUwb::Update(){}

    void GazeboRosUwb::ParseBeaconMapFile(std::string f)
    {
    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboRosUwb)
}

