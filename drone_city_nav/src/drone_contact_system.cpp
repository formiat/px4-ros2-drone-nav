#include <chrono>
#include <cstdint>
#include <gz/common/Console.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/transport/Node.hh>
#include <memory>
#include <sdf/Element.hh>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace drone_city_nav {

class DroneContactSystem final : public gz::sim::System,
                                 public gz::sim::ISystemConfigure,
                                 public gz::sim::ISystemPostUpdate {
public:
  void Configure(const gz::sim::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm,
                 gz::sim::EventManager& event_manager) override {
    static_cast<void>(event_manager);
    const gz::sim::Model model{entity};
    topic_ = sdf->Get<std::string>("topic", "/drone_city_nav/drone_contacts").first;

    for (const gz::sim::Entity link_entity : model.Links(ecm)) {
      const gz::sim::Link link{link_entity};
      for (const gz::sim::Entity collision_entity : link.Collisions(ecm)) {
        drone_collisions_.push_back(collision_entity);
        drone_collision_ids_.insert(collision_entity);
        if (ecm.Component<gz::sim::components::ContactSensorData>(collision_entity) ==
            nullptr) {
          ecm.CreateComponent(collision_entity,
                              gz::sim::components::ContactSensorData{});
        }
      }
    }

    publisher_ = node_.Advertise<gz::msgs::Contacts>(topic_);
    gzmsg << "DroneContactSystem monitoring " << drone_collisions_.size()
          << " collision entities on [" << topic_ << "]\n";
  }

  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& ecm) override {
    if (info.paused || drone_collisions_.empty()) {
      return;
    }

    gz::msgs::Contacts output;
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(info.simTime);
    const auto nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime - seconds);
    output.mutable_header()->mutable_stamp()->set_sec(seconds.count());
    output.mutable_header()->mutable_stamp()->set_nsec(
        static_cast<std::int32_t>(nanoseconds.count()));

    std::unordered_set<std::string> emitted_pairs;
    for (const gz::sim::Entity collision_entity : drone_collisions_) {
      const auto* data =
          ecm.Component<gz::sim::components::ContactSensorData>(collision_entity);
      if (data == nullptr) {
        continue;
      }

      for (const auto& contact : data->Data().contact()) {
        const bool first_is_drone =
            drone_collision_ids_.contains(contact.collision1().id());
        const bool second_is_drone =
            drone_collision_ids_.contains(contact.collision2().id());
        if (first_is_drone == second_is_drone) {
          continue;
        }

        const std::uint64_t drone_id =
            first_is_drone ? contact.collision1().id() : contact.collision2().id();
        const std::uint64_t obstacle_id =
            first_is_drone ? contact.collision2().id() : contact.collision1().id();
        const std::string pair_key =
            std::to_string(drone_id) + ":" + std::to_string(obstacle_id);
        if (!emitted_pairs.insert(pair_key).second) {
          continue;
        }

        auto* normalized = output.add_contact();
        *normalized = contact;
        if (!first_is_drone) {
          *normalized->mutable_collision1() = contact.collision2();
          *normalized->mutable_collision2() = contact.collision1();
        }
      }
    }

    if (output.contact_size() > 0) {
      publisher_.Publish(output);
    }
  }

private:
  std::string topic_;
  std::vector<gz::sim::Entity> drone_collisions_;
  std::unordered_set<gz::sim::Entity> drone_collision_ids_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
};

} // namespace drone_city_nav

GZ_ADD_PLUGIN(drone_city_nav::DroneContactSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
