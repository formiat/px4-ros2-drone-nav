#pragma once

#include <mutex>

namespace drone_city_nav {

// Ownership of the execution-evidence transaction, independent of ROS.
//
// A controller-visible publication may only claim evidence that was coherent at
// the moment it was committed. Several independent producers can invalidate
// that evidence: raw-world replacement, route activation, horizon commit,
// execution holds, and latest-sensor admission. Previously each of those sites
// took the mutexes it happened to need, so the lock order and the atomicity of
// a commit were a convention spread across the ROS node rather than a property
// of any one type.
//
// This boundary owns the mutexes and hands out the exact scopes that exist.
// Adding a new combination means adding a scope here, where the ordering is
// visible, instead of composing one at a call site.
class ExecutionEvidenceBoundary3D final {
public:
  // Raw ROS inputs: navigation, vehicle status, control feedback, objective.
  class InputScope final {
  public:
    explicit InputScope(std::mutex& input)
        : lock_{input} {
    }

  private:
    std::scoped_lock<std::mutex> lock_;
  };

  // Pose ingestion must release raw inputs before it enqueues world work, so
  // this scope can end early. It is deliberately the only releasable scope.
  class ReleasableInputScope final {
  public:
    explicit ReleasableInputScope(std::mutex& input)
        : lock_{input} {
    }

    void release() {
      lock_.unlock();
    }

  private:
    std::unique_lock<std::mutex> lock_;
  };

  // An objective transition that also moves the replan anchor.
  class InputObjectiveReplanScope final {
  public:
    InputObjectiveReplanScope(std::mutex& input, std::mutex& objective_replan)
        : lock_{input, objective_replan} {
    }

  private:
    std::scoped_lock<std::mutex, std::mutex> lock_;
  };

  // Evidence a publication may claim, without touching raw inputs.
  class EvidenceScope final {
  public:
    explicit EvidenceScope(std::mutex& evidence)
        : lock_{evidence} {
    }

  private:
    std::scoped_lock<std::mutex> lock_;
  };

  // A commit that reads raw inputs and seals evidence in one transaction.
  class EvidenceInputScope final {
  public:
    EvidenceInputScope(std::mutex& evidence, std::mutex& input)
        : lock_{evidence, input} {
    }

  private:
    std::scoped_lock<std::mutex, std::mutex> lock_;
  };

  // Active publication validates raw-world and latest-sensor admission together.
  class EvidenceLatestSensorScope final {
  public:
    EvidenceLatestSensorScope(std::mutex& evidence, std::mutex& latest_sensor)
        : lock_{evidence, latest_sensor} {
    }

  private:
    std::scoped_lock<std::mutex, std::mutex> lock_;
  };

  // Latest-sensor admission alone, independent of raw-world reconstruction.
  class LatestSensorScope final {
  public:
    explicit LatestSensorScope(std::mutex& latest_sensor)
        : lock_{latest_sensor} {
    }

  private:
    std::scoped_lock<std::mutex> lock_;
  };

  [[nodiscard]] InputScope input() {
    return InputScope{input_mutex_};
  }

  [[nodiscard]] ReleasableInputScope releasableInput() {
    return ReleasableInputScope{input_mutex_};
  }

  [[nodiscard]] InputObjectiveReplanScope inputWithObjectiveReplan() {
    return InputObjectiveReplanScope{input_mutex_, objective_replan_mutex_};
  }

  [[nodiscard]] EvidenceScope evidence() {
    return EvidenceScope{execution_evidence_commit_mutex_};
  }

  [[nodiscard]] EvidenceInputScope evidenceWithInput() {
    return EvidenceInputScope{execution_evidence_commit_mutex_, input_mutex_};
  }

  [[nodiscard]] EvidenceLatestSensorScope evidenceWithLatestSensor() {
    return EvidenceLatestSensorScope{execution_evidence_commit_mutex_,
                                     latest_sensor_evidence_commit_mutex_};
  }

  [[nodiscard]] LatestSensorScope latestSensor() {
    return LatestSensorScope{latest_sensor_evidence_commit_mutex_};
  }

private:
  std::mutex execution_evidence_commit_mutex_;
  // Latest-sensor admission is independent from raw-world reconstruction. Active
  // execution publication locks both domains to validate one coherent boundary.
  std::mutex latest_sensor_evidence_commit_mutex_;
  std::mutex input_mutex_;
  std::mutex objective_replan_mutex_;
};

} // namespace drone_city_nav
