#pragma once

struct SimulationBatchRequest {
  SimulationBatchInput input{};
  cudaStream_t stream{nullptr};
  cudaEvent_t noise_ready{nullptr};
  cudaEvent_t simulation_done{nullptr};
  std::size_t rollout_blocks{0U};
  std::size_t executed_batch_size{0U};
  std::exception_ptr error;
  bool claimed{false};
  bool completed{false};
};

class SimulationBatchCoordinator {
public:
  static constexpr std::size_t kMaximumBatchSize{8U};

  SimulationBatchCoordinator()
      : inputs_{kMaximumBatchSize} {
    checkCuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking),
              "create MPPI batch stream");
  }

  SimulationBatchCoordinator(const SimulationBatchCoordinator&) = delete;
  SimulationBatchCoordinator& operator=(const SimulationBatchCoordinator&) = delete;

  ~SimulationBatchCoordinator() {
    if (stream_ != nullptr) {
      (void)cudaStreamDestroy(stream_);
    }
  }

  [[nodiscard]] std::size_t submit(SimulationBatchRequest& request,
                                   const std::size_t desired_batch_size) {
    if (desired_batch_size < 2U || desired_batch_size > kMaximumBatchSize) {
      throw std::invalid_argument{"invalid MPPI simulation batch size"};
    }

    std::vector<SimulationBatchRequest*> batch;
    {
      std::unique_lock lock{queue_mutex_};
      waiting_.push_back(&request);
      const auto deadline =
          std::chrono::steady_clock::now() + std::chrono::milliseconds{2};
      while (!request.claimed) {
        if (waiting_.size() >= desired_batch_size) {
          claimWaiting(batch);
          break;
        }
        if (queue_condition_.wait_until(lock, deadline) == std::cv_status::timeout) {
          if (!request.claimed) {
            claimWaiting(batch);
          }
          break;
        }
      }
      if (batch.empty()) {
        queue_condition_.wait(lock, [&request]() { return request.completed; });
        if (request.error) {
          std::rethrow_exception(request.error);
        }
        return request.executed_batch_size;
      }
    }

    std::exception_ptr error;
    try {
      execute(batch);
    } catch (...) {
      error = std::current_exception();
    }
    {
      const std::scoped_lock lock{queue_mutex_};
      for (SimulationBatchRequest* const pending : batch) {
        pending->executed_batch_size = batch.size();
        pending->error = error;
        pending->completed = true;
      }
    }
    queue_condition_.notify_all();
    if (error) {
      std::rethrow_exception(error);
    }
    return batch.size();
  }

private:
  void claimWaiting(std::vector<SimulationBatchRequest*>& batch) {
    batch.swap(waiting_);
    for (SimulationBatchRequest* const pending : batch) {
      pending->claimed = true;
    }
    queue_condition_.notify_all();
  }

  void execute(const std::vector<SimulationBatchRequest*>& batch) {
    const std::scoped_lock execution_lock{execution_mutex_};
    std::vector<SimulationBatchInput> host_inputs;
    host_inputs.reserve(batch.size());
    std::size_t maximum_rollout_blocks{0U};
    for (const SimulationBatchRequest* const pending : batch) {
      host_inputs.push_back(pending->input);
      maximum_rollout_blocks =
          std::max(maximum_rollout_blocks, pending->rollout_blocks);
      checkCuda(cudaStreamWaitEvent(stream_, pending->noise_ready, 0U),
                "wait for MPPI batch noise");
    }
    checkCuda(cudaMemcpyAsync(inputs_.get(), host_inputs.data(),
                              host_inputs.size() * sizeof(SimulationBatchInput),
                              cudaMemcpyHostToDevice, stream_),
              "upload MPPI simulation batch");
    const dim3 blocks{static_cast<unsigned int>(maximum_rollout_blocks),
                      static_cast<unsigned int>(batch.size()), 1U};
    simulateBatch<<<blocks, kThreadsPerBlock, 0U, stream_>>>(inputs_.get(),
                                                             batch.size());
    checkCuda(cudaPeekAtLastError(), "launch MPPI simulation batch");
    batch_done_.record(stream_);
    for (const SimulationBatchRequest* const pending : batch) {
      checkCuda(cudaStreamWaitEvent(pending->stream, batch_done_.get(), 0U),
                "wait for MPPI simulation batch");
      checkCuda(cudaEventRecord(pending->simulation_done, pending->stream),
                "record MPPI simulation completion");
    }
  }

  DeviceBuffer<SimulationBatchInput> inputs_;
  cudaStream_t stream_{nullptr};
  Event batch_done_;
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  std::vector<SimulationBatchRequest*> waiting_;
  std::mutex execution_mutex_;
};

[[nodiscard]] inline SimulationBatchCoordinator& simulationBatchCoordinator() {
  static SimulationBatchCoordinator coordinator;
  return coordinator;
}
