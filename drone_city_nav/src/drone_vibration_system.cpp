// The rotors' vibration on the vehicle's body (roadmap item 35, specification
// K52): a wrench at the rotors' frequency applied to base_link every physics
// step, so that the IMU the autopilot and the estimators read sees what a real
// quadrotor's IMU sees. The amplitudes are accelerations and angular rates
// measured on a real recording (NTU VIRAL eee_01: a peak at 55 Hz, 0.14, 0.39,
// 0.27 m/s^2 rms above 50 Hz along x, y, z and 0.011, 0.010, 0.003 rad/s rms;
// `tools/imperfections_sources.py`); the plugin turns them into a force and a
// torque with the link's own mass and inertia, which the preparation reads off
// the model, and the seed sets the phases. A simulation component: it lives in
// the runtime copy of the vehicle model when `IMPERFECTIONS=nominal` and in no
// onboard image.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <gz/common/Console.hh>
#include <gz/math/Inertial.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Inertial.hh>
#include <memory>
#include <numbers>
#include <sdf/Element.hh>
#include <string>

namespace drone_city_nav {

class DroneVibrationSystem final : public gz::sim::System,
                                   public gz::sim::ISystemConfigure,
                                   public gz::sim::ISystemPreUpdate {
public:
  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& event_manager) override {
    static_cast<void>(event_manager);
    const gz::sim::Model model{entity};
    const std::string link_name = sdf->Get<std::string>("link", "base_link").first;
    link_ = gz::sim::Link{model.LinkByName(ecm, link_name)};
    frequency_hz_ = sdf->Get<double>("frequency_hz", 55.0).first;
    acceleration_mps2_ =
        sdf->Get<gz::math::Vector3d>("acceleration_amplitude_mps2",
                                     gz::math::Vector3d{0.192, 0.552, 0.380})
            .first;
    rate_radps_ =
        sdf->Get<gz::math::Vector3d>("rate_amplitude_radps",
                                     gz::math::Vector3d{0.0156, 0.0141, 0.0042})
            .first;
    const auto seed = static_cast<std::uint32_t>(sdf->Get<int>("seed", 0).first);
    // Three phases from the seed: the axes of a real airframe do not share one.
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
      const std::uint32_t hash =
          (seed + 1U) * 2654435761U + static_cast<std::uint32_t>(axis) * 40503U;
      phase_.at(axis) =
          2.0 * std::numbers::pi * static_cast<double>(hash % 1000U) / 1000.0;
    }
    if (!link_.Valid(ecm)) {
      gzerr << "DroneVibrationSystem: no link '" << link_name << "' in the model\n";
      return;
    }
    link_.EnableVelocityChecks(ecm, true);
    const auto* inertial = ecm.Component<gz::sim::components::Inertial>(link_.Entity());
    if (inertial == nullptr) {
      gzerr << "DroneVibrationSystem: link '" << link_name << "' has no inertial\n";
      return;
    }
    mass_kg_ = inertial->Data().MassMatrix().Mass();
    moments_kgm2_ = inertial->Data().MassMatrix().DiagonalMoments();
    ready_ = mass_kg_ > 0.0;
    gzmsg << "DroneVibrationSystem on [" << link_name << "] at " << frequency_hz_
          << " Hz: acceleration " << acceleration_mps2_ << " m/s^2, rate "
          << rate_radps_ << " rad/s, mass " << mass_kg_ << " kg, moments "
          << moments_kgm2_ << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override {
    if (info.paused || !ready_) {
      return;
    }
    const std::optional<gz::math::Pose3d> pose = link_.WorldPose(ecm);
    if (!pose.has_value()) {
      return;
    }
    const double t = std::chrono::duration<double>(info.simTime).count();
    const double omega = 2.0 * std::numbers::pi * frequency_hz_;
    gz::math::Vector3d force_body;
    gz::math::Vector3d torque_body;
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
      const double angle = omega * t + phase_.at(axis);
      // A force that gives the measured acceleration; a torque that gives the
      // measured angular rate as a sinusoid, whose angular acceleration is
      // omega times the rate's amplitude.
      force_body[axis] = mass_kg_ * acceleration_mps2_[axis] * std::sin(angle);
      torque_body[axis] =
          moments_kgm2_[axis] * omega * rate_radps_[axis] * std::cos(angle);
    }
    link_.AddWorldWrench(ecm, pose->Rot().RotateVector(force_body),
                         pose->Rot().RotateVector(torque_body));
  }

private:
  gz::sim::Link link_{gz::sim::kNullEntity};
  double frequency_hz_{55.0};
  gz::math::Vector3d acceleration_mps2_;
  gz::math::Vector3d rate_radps_;
  std::array<double, 3> phase_{0.0, 0.0, 0.0};
  double mass_kg_{0.0};
  gz::math::Vector3d moments_kgm2_;
  bool ready_{false};
};

} // namespace drone_city_nav

GZ_ADD_PLUGIN(drone_city_nav::DroneVibrationSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
