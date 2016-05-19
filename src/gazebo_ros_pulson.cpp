#include <gazebo_ros_pulson/gazebo_ros_pulson.hpp>

namespace gazebo
{

    GazeboRosUwb::GazeboRosUwb(){}

    GazeboRosUwb::~GazeboRosUwb(){}

    void GazeboRosUwb::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

        world = _model->GetWorld();

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
            link = _model->GetLink();
            link_name_ = link->GetName();
        }
        else
        {
            link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
            link = _model->GetLink(link_name_);
        }

        if (!link)
        {
            ROS_FATAL("GazeboRosUwb plugin error bodyName: %s does not exist\n", link_name_.c_str());
            return;
        }

        ////////////////////////// defaults ///////////////////////////////

    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboRosUwb)
}

