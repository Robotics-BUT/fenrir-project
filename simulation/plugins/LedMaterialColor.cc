// LedMaterialColor — recolour named visuals at runtime from
// gz.msgs.MaterialColor, and (for GUI visibility) also draw a coloured BOX
// marker on each LED via the GUI MarkerManager `/marker` service.
//
// gz-sim 8.11 (vendored with ROS 2 Jazzy) ships no upstream `MaterialColor`
// system, and its GUI client does not reliably repaint a visual's Material at
// runtime. So this system does two things when a colour arrives:
//   1. sets the matching visual's Material (ambient/diffuse/emissive) — the
//      "on-model" recolour (shows in sensor renders / gz versions that repaint);
//   2. requests a BOX marker at that visual's world pose with the same colour —
//      which the GUI MarkerManager renders reliably, so the LEDs visibly change
//      colour in the Gazebo window.
//
// Wiring: /bpc_prp_robot/rgb_leds -> rgb_leds_bridge.py -> 4x
//   ros_gz_interfaces/MaterialColor on /led_colors -> ros_gz_bridge ->
//   gz.msgs.MaterialColor on /led_colors -> this system.
//
// SDF params (optional):
//   <topic>         gz topic to subscribe (default "/led_colors")
//   <marker_topic>  GUI marker service (default "/marker")
//   <markers>       "true"/"false" — publish GUI markers (default true)

#include <mutex>
#include <string>
#include <unordered_map>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/msgs/empty.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/material_color.pb.h>
#include <gz/msgs/Utility.hh>

#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Material.hh>

#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Visual.hh>

namespace fenrir_sim
{
class LedMaterialColor
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override
  {
    std::string topic = "/led_colors";
    if (_sdf->HasElement("topic"))
      topic = _sdf->Get<std::string>("topic");
    if (_sdf->HasElement("marker_topic"))
      this->markerService = _sdf->Get<std::string>("marker_topic");
    if (_sdf->HasElement("markers"))
      this->useMarkers = _sdf->Get<bool>("markers");

    this->node.Subscribe(topic, &LedMaterialColor::OnColor, this);
    gzmsg << "[LedMaterialColor] listening on [" << topic << "], markers="
          << (this->useMarkers ? "on" : "off") << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->pending.empty())
      return;

    // Substring match tolerates gz's fixed-joint lumping, which can rename
    // "led_front" to "<...>_fixed_joint_lump__led_front_visual".
    _ecm.Each<gz::sim::components::Name, gz::sim::components::Material>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Name *_name,
            gz::sim::components::Material *) -> bool
        {
          for (const auto &kv : this->pending)
          {
            if (_name->Data().find(kv.first) == std::string::npos)
              continue;

            // (1) recolour the on-model visual
            sdf::Material mat;
            mat.SetAmbient(kv.second);
            mat.SetDiffuse(kv.second);
            mat.SetEmissive(kv.second);
            _ecm.SetComponentData<gz::sim::components::Material>(_entity, mat);
            _ecm.SetChanged(_entity,
                            gz::sim::components::Material::typeId,
                            gz::sim::ComponentState::OneTimeChange);

            // (2) draw a coloured marker at the visual's world pose
            if (this->useMarkers)
              this->SendMarker(this->MarkerId(kv.first),
                               gz::sim::worldPose(_entity, _ecm), kv.second);
            break;
          }
          return true;
        });

    this->pending.clear();
  }

private:
  void OnColor(const gz::msgs::MaterialColor &_msg)
  {
    if (_msg.entity().name().empty())
      return;
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pending[_msg.entity().name()] = gz::msgs::Convert(_msg.diffuse());
  }

  static int MarkerId(const std::string &_key)
  {
    if (_key.find("front") != std::string::npos) return 1;
    if (_key.find("right") != std::string::npos) return 2;
    if (_key.find("back")  != std::string::npos) return 3;
    if (_key.find("left")  != std::string::npos) return 4;
    return 0;
  }

  void SendMarker(int _id, const gz::math::Pose3d &_pose,
                  const gz::math::Color &_color)
  {
    gz::msgs::Marker m;
    m.set_ns("fenrir_leds");
    m.set_id(_id);
    m.set_action(gz::msgs::Marker::ADD_MODIFY);
    m.set_type(gz::msgs::Marker::BOX);
    gz::msgs::Set(m.mutable_pose(), _pose);
    gz::msgs::Set(m.mutable_scale(), gz::math::Vector3d(0.0225, 0.0225, 0.015));
    auto *mat = m.mutable_material();
    gz::msgs::Set(mat->mutable_ambient(), _color);
    gz::msgs::Set(mat->mutable_diffuse(), _color);
    gz::msgs::Set(mat->mutable_emissive(), _color);
    // Fire-and-forget async request so PreUpdate never blocks on the GUI.
    this->node.Request(this->markerService, m, this->markerCb);
  }

  gz::transport::Node node;
  std::mutex mutex;
  std::unordered_map<std::string, gz::math::Color> pending;
  std::string markerService{"/marker"};
  bool useMarkers{true};
  std::function<void(const gz::msgs::Empty &, const bool)> markerCb =
      [](const gz::msgs::Empty &, const bool) {};
};
}  // namespace fenrir_sim

GZ_ADD_PLUGIN(fenrir_sim::LedMaterialColor,
              gz::sim::System,
              fenrir_sim::LedMaterialColor::ISystemConfigure,
              fenrir_sim::LedMaterialColor::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(fenrir_sim::LedMaterialColor, "fenrir_sim::LedMaterialColor")
