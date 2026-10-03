// record_frames OUT_DIR [WORLD MODEL]: every frame of the stereo pair over gz-transport, grey uint8, into preallocated files with
// direct I/O (left.bin, right.bin) and frames.csv: side,stamp_s,wall_s,width,height,byte_offset. The Python recorder took the
// images under the interpreter's lock and the simulator stalled behind it (r549: every ROS node silent for 0.6 s, PX4's IMU lost
// 0.7 s); here a frame costs a conversion and one write on the transport's own thread.
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
struct Side { int fd{-1}; long long offset{0}; unsigned char* block{nullptr}; std::size_t capacity{0}; long long frames{0}; };
int main(int argc, char** argv) {
  const std::string out = argv[1];
  const std::string world = argc > 2 ? argv[2] : "urban_circuit_practice_01_collisions";
  const std::string model = argc > 3 ? argv[3] : "x500_lidar_3d_0";
  std::FILE* index = std::fopen((out + "/frames.csv").c_str(), "w");
  std::fprintf(index, "side,stamp_s,wall_s,width,height,byte_offset\n");
  std::mutex index_mutex;
  Side sides[2]; const char* names[2] = {"left", "right"};
  gz::transport::Node node;
  for (int s = 0; s < 2; ++s) {
    sides[s].fd = ::open((out + "/" + names[s] + ".bin").c_str(), O_WRONLY | O_CREAT | O_DIRECT, 0644);
    if (sides[s].fd < 0 || ::posix_fallocate(sides[s].fd, 0, 7LL << 30) != 0) { std::perror("open/fallocate"); return 1; }
    const std::string topic = "/world/" + world + "/model/" + model + "/link/stereo_tof_link/sensor/stereo_" + names[s] + "/image";
    const std::function<void(const gz::msgs::Image&)> callback = [&, s](const gz::msgs::Image& image) {
      Side& side = sides[s];
      const std::size_t pixels = static_cast<std::size_t>(image.width()) * image.height();
      const std::size_t padded = (pixels + 4095U) / 4096U * 4096U;
      if (side.capacity != padded) { std::free(side.block); side.block = static_cast<unsigned char*>(std::aligned_alloc(4096U, padded)); std::memset(side.block, 0, padded); side.capacity = padded; }
      const auto* source = reinterpret_cast<const unsigned char*>(image.data().data());
      if (image.data().size() >= 3U * pixels) {
        for (std::size_t i = 0; i < pixels; ++i) { const unsigned char* v = source + 3U * i; side.block[i] = static_cast<unsigned char>((4899U * v[0] + 9617U * v[1] + 1868U * v[2] + 8192U) >> 14U); }
      } else if (image.data().size() >= pixels) { std::memcpy(side.block, source, pixels); } else { return; }
      if (::pwrite(side.fd, side.block, padded, side.offset) != static_cast<ssize_t>(padded)) { std::perror("pwrite"); return; }
      const double wall = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
      { std::lock_guard<std::mutex> lock{index_mutex};
        std::fprintf(index, "%s,%.9f,%.6f,%u,%u,%lld\n", names[s], image.header().stamp().sec() + 1e-9 * image.header().stamp().nsec(), wall, image.width(), image.height(), side.offset);
        std::fflush(index); }
      side.offset += static_cast<long long>(padded); ++side.frames;
    };
    std::printf("%s %d\n", topic.c_str(), node.Subscribe(topic, callback) ? 1 : 0);
  }
  while (true) { std::this_thread::sleep_for(std::chrono::seconds(10)); std::printf("left %lld right %lld\n", sides[0].frames, sides[1].frames); std::fflush(stdout); }
}
