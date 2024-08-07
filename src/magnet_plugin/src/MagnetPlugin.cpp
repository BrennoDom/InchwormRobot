#include "magnet_plugin/MagnetPlugin.hpp"

#include <string>
#include <map>

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Joint.hh>

#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/JointTransmittedWrench.hh>

#include <ignition/transport/Node.hh>
#include <ignition/msgs/visual.pb.h>

#include<vector> 

using namespace ignition;
using namespace gazebo;
using namespace systems;

namespace magnet_plugin
{

class MagnetPluginPrivate
{
    /// \brief Initialize the plugin
    public: void Load(const EntityComponentManager &_ecm);

    /// \brief Callback for switch topic
    public: void OnSwitch(const msgs::Boolean &_msg);

    /// \brief updates all sources visual to color
    public: bool update_sources_visual(msgs::Color color, const EntityComponentManager &_ecm);

    /// \brief Send visual msg request to update the Entity's color 
    public: bool send_visual_msg(Entity id, msgs::Color &color, const EntityComponentManager &_ecm);

    /// \brief Send visual msg request to update the Entity's color 
    public: static msgs::Color create_color(float r, float g, float b, float a);

    /// \brief Send visual msg request to update the Entity's color 
    public: void remove_detachable_joints(EntityComponentManager &_ecm);

    /// \brief Updates the status topic
    public: void update_status_topic(const UpdateInfo &_info);

    /// \brief Updates the connection topic
    public: void update_connection_topic(const UpdateInfo &_info);

    /// \brief source's link_name parameter defined in the sdf file
    public: std::vector<std::string> source_link_names;
    
    /// \brief source's collision_name parameter defined in the sdf file
    public: std::vector<std::string> source_collision_names;

    /// \brief target's model_name parameter defined in the sdf file
    public: std::string target_model_name;
    
    /// \brief target's link_name parameter defined in the sdf file
    public: std::string target_link_name;
    
    /// \brief target's collision_name parameter defined in the sdf file
    public: std::string target_collision_name;

    /// \brief switch topic defined in the sdf file
    public: std::string switch_topic;

    /// \brief connection topic defined in the sdf file
    public: std::string connection_topic;

    /// \brief status topic defined in the sdf file
    public: std::string status_topic;

    /// \brief visual option defined in the sdf file
    public: bool visual;

    /// \brief joint used as reference for force
    public: std::string reference_joint_name;

    /// \brief if true, turn of the connection when reach the limits
    public: bool reference_disconnect;

    /// \brief joint used as reference for force
    public: Entity reference_joint_entity{kNullEntity};

    /// \brief joint used as reference for force
    public: math::Vector3d reference_min_force;

    /// \brief joint used as reference for force
    public: math::Vector3d reference_max_force;
    
    /// \brief Source entity (Model)
    public: Entity source_entity{kNullEntity};

    /// \brief Source Model
    public: Model source_model;

    /// \brief List of source Link 
    public: std::vector<Link> source_links;

    /// \brief List of source colision entity used to filter collisions
    public: std::vector<Entity> source_colision_entities;

    /// \brief Source colision entity used to filter collisions
    public: Entity target_colision_entity{kNullEntity};

    /// \brief Target Link 
    public: Link target_link;

    /// \brief Initialization flag
    public: bool initialized{false};

    /// \brief switch request sent by switch_topic
    public: bool switch_request{false};

    /// \brief connection status published on connection_topic
    public: bool connection_status{false};

    /// \brief Ing communication node.
    public: transport::Node node;

    /// \brief Ing joint created at eecution time.
    public: std::vector<Entity> detachable_joints;

    /// \brief Ign transport publisher
    public: transport::Node::Publisher connection_pub;

    /// \brief Ign transport publisher
    public: transport::Node::Publisher status_pub;
    
};

bool MagnetPluginPrivate::MagnetPluginPrivate::update_sources_visual(msgs::Color color, const EntityComponentManager &_ecm)
{
    if (!this->visual)
        return true;

    msgs::Boolean reply;
    bool result = true;

    for (auto s : this->source_links)
    {
        //Considering that each link has one visual element
        auto id_visual = s.Visuals(_ecm)[0]; 
        result = result && send_visual_msg(id_visual, color, _ecm);

    }

    return result;
}

bool MagnetPluginPrivate::MagnetPluginPrivate::send_visual_msg(Entity id, msgs::Color &color, const EntityComponentManager &_ecm)
{
    auto visual_topic = scopedName(1,_ecm) + "/visual_config";
    msgs::Boolean reply;
    bool result = false;

    auto material_msg = new msgs::Material();
    material_msg->set_allocated_diffuse(new msgs::Color(color));

    auto visual_msg = std::make_shared<msgs::Visual>();
    visual_msg->set_id((google::protobuf::uint32)id);
    visual_msg->set_allocated_material(material_msg);

    if (!this->node.Request<msgs::Visual, msgs::Boolean>(visual_topic, *(visual_msg.get()), 5000,
                                                         reply, result))
      ignerr << "Failed to find visual topic [" << visual_topic << "]" << std::endl;

    return result;
}

void MagnetPluginPrivate::Load(const EntityComponentManager &_ecm)
{
    auto potential_entities = _ecm.EntitiesByComponents(components::Link());
    std::map<std::string, Entity> link_entities_map;

    for (Entity entity : potential_entities)
    {
        std::string key = scopedName(entity, _ecm);
        auto start = key.find("/model/") + std::string("/model/").length();
        key = key.substr(start);
        link_entities_map[key] = entity;
    }

    for (uint32_t i = 0; i < this->source_link_names.size(); i++)
    {
        std::string source_search = this->source_model.Name(_ecm);
        source_search += "/link/";
        source_search += this->source_link_names[i];

        if (auto search = link_entities_map.find(source_search); search != link_entities_map.end())
        {
            auto tmp_collision = this->source_links[i].CollisionByName(_ecm, this->source_collision_names[i]);
            //auto tmp_collision = this->source_links[i].Collisions(_ecm)[0]; we could change for this...?
            if (tmp_collision == kNullEntity)
            {
                ignerr << "Failed to find source collision [" << this->source_collision_names[i] << "]" << std::endl;
                return;
            }
            this->source_colision_entities.push_back(tmp_collision);
        }
        else
        {
            ignerr << "Failed to find source link [" << this->source_link_names[i] << "]" << std::endl;
            return;
        }
    }

    std::string target_search = this->target_model_name;
    target_search += "/link/";
    target_search += this->target_link_name;

    if (auto search = link_entities_map.find(target_search); search != link_entities_map.end())
    {
        this->target_link = Link(search->second);
        auto tmp_collision = this->target_link.CollisionByName(_ecm, this->target_collision_name);
        if (tmp_collision == kNullEntity)
        {
            ignerr << "Failed to find target collision [" << this->target_collision_name << "]" << std::endl;
            return;
        }
        this->target_colision_entity = tmp_collision;
    }
    else
    {
        ignerr << "Failed to find target link [" << this->target_link_name << "]" << std::endl;
        return;
    }
        
    this->initialized = true;

    this->update_sources_visual(this->create_color(255,0,0,1), _ecm);
}


void MagnetPluginPrivate::OnSwitch(const msgs::Boolean &_msg)
{
    this->switch_request = _msg.data();
}

msgs::Color MagnetPluginPrivate::create_color(float r, float g, float b, float a)
{
    msgs::Color color;
    color.set_r(r);
    color.set_g(g);
    color.set_b(b);
    color.set_a(a);
    return color;
}

void MagnetPluginPrivate::remove_detachable_joints(EntityComponentManager &_ecm)
{
    for (auto &e : this->detachable_joints)
        _ecm.RequestRemoveEntity(e);
     
    this->detachable_joints.clear();
    this->connection_status = false;
    this->switch_request = false;
    this->update_sources_visual(this->create_color(255, 0, 0, 1), _ecm);
}

void MagnetPluginPrivate::update_connection_topic(const UpdateInfo &_info)
{
    msgs::Boolean msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
    msg.set_data(this->connection_status);
    this->connection_pub.Publish(msg);
}

void MagnetPluginPrivate::update_status_topic(const UpdateInfo &_info)
{
    msgs::Boolean msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
    msg.set_data(this->switch_request);
    this->status_pub.Publish(msg);
}

/**/
MagnetPlugin::MagnetPlugin() : dataPtr(std::make_unique<MagnetPluginPrivate>())
{
}

MagnetPlugin::~MagnetPlugin()
{
}

/* Inherited method */
void MagnetPlugin::Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager &/*_eventMgr*/)
{
    

    this->dataPtr->source_entity = _entity;
    this->dataPtr->source_model = Model(_entity);
    
    
    auto sdf = _sdf->Clone();

    for (auto source_element = sdf->GetElement("source"); 
              source_element != nullptr;
              source_element = source_element->GetNextElement("source"))
    {
        auto source_link_name = source_element->Get<std::string>("link_name");
        if (source_link_name.empty())
        {
            ignerr  << "MagnetPlugin missing source_link_name. "
                    << "Failed to initialize." << std::endl;
            return;
        }
        this->dataPtr->source_link_names.push_back(source_link_name);

        auto source_collision_name = source_element->Get<std::string>("collision_name");
        if (source_collision_name.empty())
        {
            ignerr  << "MagnetPlugin missing source_collision_name. "
                    << "Failed to initialize." << std::endl;
            return;
        }
        this->dataPtr->source_collision_names.push_back(source_collision_name);

        this->dataPtr->source_links.push_back(Link(this->dataPtr->source_model.LinkByName(_ecm, source_link_name)));
        this->dataPtr->source_links.back().EnableVelocityChecks(_ecm); //adding WordPose internaly            
    }

    auto target_element = sdf->GetElement("target");
    this->dataPtr->target_model_name = target_element->Get<std::string>("model_name");
    if (this->dataPtr->target_model_name.empty())
    {
        ignerr  << "MagnetPlugin missing target_model_name. "
                << "Failed to initialize." << std::endl;
        return;
    }
    this->dataPtr->target_link_name = target_element->Get<std::string>("link_name");
    if (this->dataPtr->target_link_name.empty())
    {
        ignerr  << "MagnetPlugin missing target_link_name. "
                << "Failed to initialize." << std::endl;
        return;
    }
    this->dataPtr->target_collision_name = target_element->Get<std::string>("collision_name");
    if (this->dataPtr->target_collision_name.empty())
    {
        ignerr  << "MagnetPlugin missing target_collision_name. "
                << "Failed to initialize." << std::endl;
        return;
    }

    std::string default_prefix = "/model/" + this->dataPtr->source_model.Name(_ecm) + "/";
    
    std::string default_switch = default_prefix + "magnet_plugin/switch";
    this->dataPtr->switch_topic = sdf->Get<std::string>("switch_topic", default_switch).first;
    
    if (this->dataPtr->switch_topic[0] != '/')
        this->dataPtr->switch_topic = default_prefix + this->dataPtr->switch_topic;

    if (this->dataPtr->switch_topic.empty())
    {
        ignerr  << "MagnetPlugin missing switch_topic. "
                << "Failed to initialize." << std::endl;
        return;
    }

    std::string default_connection = default_prefix + "magnet_plugin/connection";
    this->dataPtr->connection_topic = sdf->Get<std::string>("connection_topic", default_connection).first;
    if (this->dataPtr->connection_topic[0] != '/')
        this->dataPtr->connection_topic = default_prefix + this->dataPtr->connection_topic;

    std::string default_status = default_prefix + "magnet_plugin/status";
    this->dataPtr->status_topic = sdf->Get<std::string>("status_topic", default_status).first;
    if (this->dataPtr->status_topic[0] != '/')
        this->dataPtr->status_topic = default_prefix + this->dataPtr->status_topic;

    auto reference_element = sdf->FindElement("reference");
    
    if(reference_element)
    {
        this->dataPtr->reference_joint_name = reference_element->Get<std::string>("joint_name","").first;
        if (this->dataPtr->reference_joint_name.empty())
        {
            ignerr  << "MagnetPlugin missing reference [joint_name]. "
                    << "Failed to initialize." << std::endl;
            return;
        }
        this->dataPtr->reference_joint_entity = this->dataPtr->source_model.JointByName(_ecm, this->dataPtr->reference_joint_name);
        if (this->dataPtr->reference_joint_entity == kNullEntity)
        {
            ignerr  << "MagnetPlugin not found [" << this->dataPtr->reference_joint_name << "]. "
                    << "Failed to initialize." << std::endl;
            return; 
        }
        this->dataPtr->reference_disconnect = reference_element->Get<bool>("disconnect", true).first;
        math::Vector3d min_default(-math::INF_D,-math::INF_D,-math::INF_D);
        this->dataPtr->reference_min_force = reference_element->Get<math::Vector3d>("min_force",min_default).first;
        math::Vector3d max_default(math::INF_D,math::INF_D,math::INF_D);
        this->dataPtr->reference_max_force = reference_element->Get<math::Vector3d>("max_force",max_default).first;
    }



    this->dataPtr->connection_pub = this->dataPtr->node.Advertise<msgs::Boolean>(this->dataPtr->connection_topic);
    if (!this->dataPtr->connection_pub)
    {
        ignerr << "Publisher could not be created for ["<< this->dataPtr->connection_topic <<"] topic." << std::endl;
        return;
    }

    this->dataPtr->status_pub = this->dataPtr->node.Advertise<msgs::Boolean>(this->dataPtr->status_topic);
    if (!this->dataPtr->status_pub)
    {
        ignerr << "Publisher could not be created for ["<< this->dataPtr->status_pub <<"] topic." << std::endl;
        return;
    }

    if (!this->dataPtr->node.Subscribe(this->dataPtr->switch_topic, &MagnetPluginPrivate::OnSwitch, this->dataPtr.get()))
    {
        ignerr << "Subscriber could not be created for ["<< this->dataPtr->switch_topic <<"] topic." << std::endl;
        return;
    }

    this->dataPtr->visual = sdf->Get<bool>("visual", true).first;

    //ignmsg << "source_link_name defined as " << this->dataPtr->source_link_name << std::endl;
    //ignmsg << "source_collision_name defined as " << this->dataPtr->source_collision_name << std::endl;
    for (auto &name: this->dataPtr->source_link_names)
        ignmsg << "source_link_name defined as " << name << std::endl; 
    
    for (auto &name: this->dataPtr->source_collision_names)
        ignmsg << "source_collision_name defined as " << name << std::endl;

    ignmsg << "target_model_name defined as " << this->dataPtr->target_model_name << std::endl;
    ignmsg << "target_link_name defined as " << this->dataPtr->target_link_name << std::endl;
    ignmsg << "target_collision_name defined as " << this->dataPtr->target_collision_name << std::endl;
    ignmsg << "switch_topic defined as " << this->dataPtr->switch_topic << std::endl;
    ignmsg << "visual defined as " << this->dataPtr->visual << std::endl;
}

/* Inherited method */
void MagnetPlugin::PreUpdate(const UpdateInfo &_info,
                             EntityComponentManager &_ecm)
{
    if (_info.paused)
        return;

    if (!this->dataPtr->initialized)
    {
        //I call it here because it is not guaranteed that all
        //entities have been created when Configure is called
        this->dataPtr->Load(_ecm);
        return;
    }
    
    if (_info.iterations <= 5) //For when the simulation starts paused
        return;

    if (this->dataPtr->switch_request != this->dataPtr->connection_status)
    {
        if (this->dataPtr->switch_request == false)
            this->dataPtr->remove_detachable_joints(_ecm);

        if (this->dataPtr->switch_request == true)
        {
            //Checking if every source has a contact
            uint32_t sources_with_contact = 0;
            for (auto &e : this->dataPtr->source_colision_entities)
            {
                auto *tmp_contacts = _ecm.Component<components::ContactSensorData>(e);
                if (tmp_contacts)
                {
                    for (const auto &contact : tmp_contacts->Data().contact())
                    {
                        if ((contact.collision2().id() == this->dataPtr->target_colision_entity) && 
                            (contact.position_size() > 0))
                            {
                                sources_with_contact += 1;
                                break;
                            }                              
                    }
                }
                else
                    break;
            }

            //if every source has a contact, create the joint
            if (sources_with_contact == this->dataPtr->source_colision_entities.size())
            {
                this->dataPtr->connection_status = true;
                for (auto &sl : this->dataPtr->source_links)
                {
                    auto tmp_entity = _ecm.CreateEntity();
                    this->dataPtr->detachable_joints.push_back(tmp_entity);
                    _ecm.CreateComponent(
                            tmp_entity,
                            components::DetachableJoint({this->dataPtr->target_link.Entity(),
                                                         sl.Entity(), "fixed"}));
                }
                this->dataPtr->update_status_topic(_info);
                this->dataPtr->update_sources_visual(this->dataPtr->create_color(0, 255, 0, 1), _ecm);
            }
        }
    }

    //If it is connected and is monitoring joint forces
    if (this->dataPtr->connection_status && this->dataPtr->reference_joint_entity != kNullEntity)
    {
        auto *force_torque = _ecm.Component<components::JointTransmittedWrench>(this->dataPtr->reference_joint_entity);
        if (force_torque) 
        {
            math::Vector3d measured_force(force_torque->Data().force().x(),
                                          force_torque->Data().force().y(),
                                          force_torque->Data().force().z());

            if ((this->dataPtr->reference_max_force < measured_force) || 
                (measured_force < this->dataPtr->reference_min_force))
            {
                ignmsg << "LOST CONTACT ! -- Measured Force (x,y,z): " << measured_force << std::endl;
                if (this->dataPtr->reference_disconnect)
                    this->dataPtr->remove_detachable_joints(_ecm);
            }

        }
    }
    this->dataPtr->update_connection_topic(_info);
    this->dataPtr->update_status_topic(_info);
}

}  // namespace magnet_plugin

// Register plugin
IGNITION_ADD_PLUGIN(magnet_plugin::MagnetPlugin,
                    System,
                    magnet_plugin::MagnetPlugin::ISystemConfigure,
                    magnet_plugin::MagnetPlugin::ISystemPreUpdate)


// Add plugin alias so that we can refer to the plugin without the version
// namespace
IGNITION_ADD_PLUGIN_ALIAS(magnet_plugin::MagnetPlugin, "ignition::gazebo::systems::MagnetPlugin")