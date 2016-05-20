#include <gazebo_ros_pulson/gazebo_ros_pulson.hpp>

namespace gazebo
{

    GazeboRosPulson::GazeboRosPulson(){}

    GazeboRosPulson::~GazeboRosPulson(){}

    void GazeboRosPulson::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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
            ROS_FATAL("GazeboRosPulson plugin error bodyName: %s does not exist\n", link_name_.c_str());
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

        std::string beacon_map_file;
        if (_sdf->HasElement("beaconMapFile"))
        {
            beacon_map_file = _sdf->GetElement("beaconMapFile")->GetValue()->GetAsString();
        }
        else
        {
            ROS_FATAL("GazeboRosPulson plugin requires a beaconMapFile\n");
            return;
        }

        int r = ParseBeaconMapFile(beacon_map_file);
        if (r != 0)
        {
            ROS_ERROR("GazeboRosPulson plugin can not open beaconMapFile\n");
            return;
        }

        range_error_model_.Load(_sdf);

        ////////////////////////// ROS ////////////////////////////////

        if (!ros::isInitialized())
        {
            ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin.\n");
            return;
        }

        nh_ = new ros::NodeHandle(namespace_);
        range_pub_ = nh_->advertise<pulson_ros::RangeMeasurement>(range_topic_, 1000);

        Reset();

        updateTimer_.setUpdateRate(120.0); // make a parameter
        updateTimer_.Load(world_, _sdf);

        /////////////////////////////////////////////////////////////////
        
        updateConnection_ = updateTimer_.Connect(boost::bind(&GazeboRosPulson::Update, this));

    }

    void GazeboRosPulson::Reset(){}

    void GazeboRosPulson::Update(){}

    int GazeboRosPulson::ParseBeaconMapFile(std::string f)
    {
        // open file
        std::fstream fs;
        fs.open(f.c_str());
        if (!fs.is_open())
            return -1;
        YAML::Node map = YAML::LoadFile(f.c_str());
        assert(map.IsSequence());

        // read beacon locations
        for (int i = 0; i < map.size(); i++)
        {
            beacon b;
            b.x = (double) map[i]["x"].as<double>();
            b.y = (double) map[i]["y"].as<double>();
            b.z = (double) map[i]["z"].as<double>();
            b.id = (int) map[i]["id"].as<int>();

            beacons_.push_back(b);
        }

        // close file
        fs.close();
        return 0;
    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboRosPulson)
}

