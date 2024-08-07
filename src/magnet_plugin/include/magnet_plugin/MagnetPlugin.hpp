#ifndef MAGNET_PLUGIN__MAGNETPLUGIN_HPP_
#define MAGNET_PLUGIN__MAGNETPLUGIN_HPP_

#include "magnet_plugin/visibility_control.h"

#include <ignition/gazebo/System.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;

namespace magnet_plugin
{

class MagnetPluginPrivate;

class MagnetPlugin:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
  public: MagnetPlugin();

  public: virtual ~MagnetPlugin();

  ///  \brief Inherited method 
  public: void Configure(const Entity &_id,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) override;

  ///  \brief Inherited method 
  public: void PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm) override;

  /// \brief Private data pointer
  private: std::unique_ptr<MagnetPluginPrivate> dataPtr;
};

}  // namespace magnet_plugin

#endif  // MAGNET_PLUGIN__MAGNETPLUGIN_HPP_
