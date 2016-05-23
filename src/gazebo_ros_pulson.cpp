#include <gazebo_ros_pulson/gazebo_ros_pulson.hpp>

namespace gazebo
{

    GazeboRosPulson::GazeboRosPulson(){}

    GazeboRosPulson::~GazeboRosPulson()
    {
        // disconnect timer
        updateTimer_.Disconnect(updateConnection_);

        // cleanup ros
        nh_->shutdown();
        delete nh_;
    }

    void GazeboRosPulson::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

        world_ = _model->GetWorld();

        // namespace
        if (!_sdf->HasElement("robotNamespace"))
        {
            namespace_.clear();
        }
        else
        {
            namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
        }

        // body name
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

        // get topic
        if (_sdf->HasElement("topicName"))
        {
            range_topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();
        }
        else
        {
            range_topic_ = "ranges";
        }

        // get update rate
        double rate;
        if (_sdf->HasElement("updateRate"))
        {
            _sdf->GetElement("updateRate")->GetValue()->Get(rate);
        }
        else
        {
            rate = 120.0;
        }

        // get node id
        if (_sdf->HasElement("nodeId"))
        {
            _sdf->GetElement("nodeId")->GetValue()->Get(node_id_);
        }
        else
        {
            node_id_ = 100;
        }

        // set counter
        counter_ = 0;

        // get beacon file
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

        // get package path
        std::string path = ros::package::getPath("gazebo_ros_pulson");

        // parse beacon map
        int r = ParseBeaconMapFile(path + "/" + "config/" + beacon_map_file);
        if (r != 0)
        {
            ROS_ERROR("GazeboRosPulson plugin can not open beaconMapFile\n");
            return;
        }

        // initialize error model
        range_error_model_.Load(_sdf);

        // ensure ros is initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin.\n");
            return;
        }

        // init node handle
        nh_ = new ros::NodeHandle(namespace_);

        // publishers
        range_pub_ = nh_->advertise<pulson_ros::RangeMeasurement>(range_topic_, 1000);

        // reset plugin
        Reset();

        // set rates
        updateTimer_.setUpdateRate(rate);
        updateTimer_.Load(world_, _sdf);

        // initialize callback
        updateConnection_ = updateTimer_.Connect(boost::bind(&GazeboRosPulson::Update, this));

        ROS_INFO("GazeboRosPulson plugin loaded!");

    }

    void GazeboRosPulson::Reset()
    {
        // reset timer
        updateTimer_.Reset();

        // reset error model
        range_error_model_.reset();
    }

    void GazeboRosPulson::Update()
    {

        // get times
        common::Time sim_time = world_->GetSimTime();
        double dt = updateTimer_.getTimeSinceLastUpdate().Double();

        // get pose
        math::Pose pose = link_->GetWorldPose();

        // get appropriate beacon
        Beacon b = beacons_.at(counter_);

        // compute range
        double r = pose.pos.Distance(b.x, b.y, b.z) * 1000.0;

        // create message
        range_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
        range_.message_id = node_id_;
        range_.responder_id = b.id;
        range_.precision_range = static_cast<unsigned int>(r);

        // publish
        range_pub_.publish(range_);

        // update counter
        counter_ = (counter_ + 1) % num_beacons_;

    }

    int GazeboRosPulson::ParseBeaconMapFile(std::string f)
    {
        ROS_INFO("Opening Beacon Map File: %s", f.c_str());

        // open file
        std::fstream fs;
        fs.open(f.c_str());
        if (!fs.is_open())
            return -1;
        YAML::Node map = YAML::LoadFile(f.c_str());
        assert(map.IsSequence());

        num_beacons_ = map.size();

        // read beacon locations
        for (int i = 0; i < num_beacons_; i++)
        {
            Beacon b;
            b.x = (double) map[i]["x"].as<double>();
            b.y = (double) map[i]["y"].as<double>();
            b.z = (double) map[i]["z"].as<double>();
            b.id = (int) map[i]["id"].as<int>();

            ROS_INFO("Beacon %d at : %f %f %f", b.id, b.x, b.y, b.z);

            beacons_.push_back(b);
        }

        // close file
        fs.close();
        return 0;
    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboRosPulson)
}

