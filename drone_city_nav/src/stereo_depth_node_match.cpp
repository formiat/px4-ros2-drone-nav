#include "stereo_depth_node.hpp"
#include "stereo_depth_node_internal.hpp"

namespace drone_city_nav {

// The frame match: the pair rectified and matched, the light the frame has left (K15),
// the silent patches and the smoke faces (K44), the ring's zones (K38, K12), the heat
// source (K49), the void (K46) and the frame's cloud (moved whole on 2026-11-02, the
// source size contract).

void StereoDepthNode::matchIfPaired() {
  if (left_ == nullptr || right_ == nullptr ||
      left_->header.stamp != right_->header.stamp) {
    return;
  }
  const sensor_msgs::msg::Image::ConstSharedPtr left = std::move(left_);
  const sensor_msgs::msg::Image::ConstSharedPtr right = std::move(right_);
  left_.reset();
  right_.reset();
  if (!usable(*left) || !usable(*right)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "STEREO_DEPTH rejected=true reason=image_layout "
                          "width=%u height=%u encoding=%s",
                          left->width, left->height, left->encoding.c_str());
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  const std::array<cv::Mat, 2> rectified_pair =
      rectifier_->rectify(stereo_depth_node_internal::grey(*left),
                          stereo_depth_node_internal::grey(*right));
  const cv::Mat& left_grey = rectified_pair[0];
  const cv::Mat& right_grey = rectified_pair[1];
  cv::Mat disparity;
  far_matcher_->compute(left_grey, right_grey, disparity);
  cv::Mat left_half;
  cv::Mat right_half;
  cv::resize(left_grey, left_half, {}, 0.5, 0.5, cv::INTER_AREA);
  cv::resize(right_grey, right_half, {}, 0.5, 0.5, cv::INTER_AREA);
  cv::Mat near_disparity;
  near_matcher_->compute(left_half, right_half, near_disparity);
  // Where the near search sees a surface beyond the far search's reach (its doubled
  // disparity within 8 px of its end) or the far found nothing, the near answer
  // stands.
  const int near_threshold_16 = 16 * (far_disparities_ - 8) / 2;
  const auto stride = static_cast<int>(returns_config_.pixel_stride);
  for (int row = 0; row < disparity.rows; row += stride) {
    for (int column = 0; column < disparity.cols; column += stride) {
      const std::int16_t near = near_disparity.at<std::int16_t>(row / 2, column / 2);
      std::int16_t& far = disparity.at<std::int16_t>(row, column);
      if (near > near_threshold_16 || far <= 0) {
        far = near > 0 ? static_cast<std::int16_t>(std::min(32767, 2 * near))
                       : std::int16_t{-16};
      }
    }
  }
  const double noise =
      stereo_depth_node_internal::rejectMatchesWithinNoise(left_grey, disparity);
  const std::array<double, LitRangeTracker::kMetres> brightness_by_metre =
      stereo_depth_node_internal::matchedBrightnessByMetre(
          left_grey, disparity, geometry_, returns_config_.pixel_stride);
  std::vector<StereoDepthReturn> returns =
      stereoDepthReturns(std::span<const std::int16_t>{disparity.ptr<std::int16_t>(),
                                                       image_width_ * image_height_},
                         image_width_, image_height_, geometry_, returns_config_);
  stereo_depth_node_internal::rejectLoneHits(returns);
  // The light the frame has left (K15): the brightness headroom, the lit range over
  // the confident depth, the light share over half its lit baseline (r1227).
  const double brightness_p95 =
      brightnessPercentile95(left_grey, stereo_depth_node_internal::kNoiseSampleStride);
  const double brightness_headroom = std::min(1.0, brightness_p95 / kGainTargetGrey);
  const bool lit = brightness_headroom > 0.9;
  const double lit_range_m = lit_range_tracker_.observe(brightness_by_metre, lit);
  double reach_m{0.0};
  for (std::size_t metre = 1U; metre < brightness_by_metre.size(); ++metre) {
    if (brightness_by_metre.at(metre) >= 0.0 &&
        brightness_by_metre.at(metre - 1U) >= 0.0) {
      reach_m = static_cast<double>(metre + 1U);
    }
  }
  const double raw_brightness =
      brightness_p95 / camera_gain_.load(std::memory_order_relaxed);
  const std::optional<double> lit_brightness = raw_brightness_baseline_.median();
  const double light_share =
      lit_brightness.has_value() && *lit_brightness > 0.0
          ? std::max(brightness_headroom > kSensorBlindFrameShare
                         ? kSensorBlindFrameShare
                         : 0.0,
                     std::min(1.0, raw_brightness / (0.5 * *lit_brightness)))
          : 1.0;
  if (lit) {
    raw_brightness_baseline_.observe(raw_brightness);
  }
  const double headroom = std::min(
      {brightness_headroom, light_share,
       std::min(1.0, lit_range_m / stereoConfidentDepthM(geometry_, returns_config_))});
  std_msgs::msg::Float64 headroom_message;
  headroom_message.data = headroom;
  light_headroom_pub_->publish(headroom_message);
  const std::size_t stereo_returns = returns.size();
  // Item 18 stage 0b: the frame's silent patches, judged on the rectified returns
  // before they turn to the physical frame; the front unit's free zones the witness.
  std::vector<Point3> free_through;
  {
    const TofSensor& front = tof_sensors_[kFrontTofSlot];
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan =
        stereo_depth_node_internal::nearestScan(front.scans,
                                                rclcpp::Time{left->header.stamp},
                                                kTofStampToleranceS, tof_mutex_);
    const std::optional<std::vector<Point3>> zones =
        scan != nullptr ? decodePointCloudReturns(*scan) : std::nullopt;
    if (zones.has_value()) {
      for (const StereoDepthReturn& ray : tofZoneReturns(*zones, front.config)) {
        if (!ray.hit) {
          free_through.push_back(rectifier_->ideal() ? ray.point
                                                     : toRectifiedFlu(ray.point));
        }
      }
    }
  }
  const std::vector<SilentPatch> patches =
      silentPatches(std::span<const StereoDepthReturn>{returns}, geometry_,
                    returns_config_, image_width_, image_height_, silent_patch_config_,
                    free_through, tof_sensors_[kFrontTofSlot].config.maximum_range_m);
  const std::size_t closed_patches = static_cast<std::size_t>(std::ranges::count_if(
      patches, [](const SilentPatch& patch) { return patch.closed; }));
  // The range a motion along the heading is bounded by (specification F13):
  // the rim's depth of a closed patch holding the frame's centre.
  double silent_range_m = silentRangeAheadM(
      patches, geometry_, returns_config_.pixel_stride,
      image_width_ / returns_config_.pixel_stride,
      image_height_ / returns_config_.pixel_stride, kSilentConeHalfAngleRad);
  std_msgs::msg::Float64 silent_message;
  silent_message.data = silent_range_m;
  silent_range_pub_->publish(silent_message);
  // The smoke faces (K44): lit silence the pair does not see through.
  const std::vector<std::uint8_t> raw_cells = rawCells(
      std::span<const std::uint8_t>{
          left_grey.data, left_grey.step[0] * static_cast<std::size_t>(left_grey.rows)},
      left_grey.step[0], image_width_, image_height_, returns_config_.pixel_stride,
      camera_gain_.load(std::memory_order_relaxed));
  const std::vector<std::uint8_t> seen_cells =
      seenCells(std::span<const StereoDepthReturn>{returns}, geometry_,
                returns_config_.pixel_stride, image_width_, image_height_);
  // The smoke face (K44) is off unless asked for (the owner, 2026-10-24).
  std::vector<SilentPatch> lit_regions =
      smoke_face_ ? smokeFaces(std::span<const StereoDepthReturn>{returns}, geometry_,
                               returns_config_, image_width_, image_height_, raw_cells,
                               silent_patch_config_, smoke_face_config_)
                  : std::vector<SilentPatch>{};
  // The plume (K44): the largest lit rimmed region of a plume's size.
  SilentPatch* candidate =
      plumeCandidate(lit_regions, silent_patch_config_, smoke_face_config_,
                     image_width_ / returns_config_.pixel_stride,
                     image_height_ / returns_config_.pixel_stride);
  const double quartile_m = candidate != nullptr ? candidate->rim_depth_m : 0.0;
  if (candidate != nullptr && std::isfinite(candidate->centre_rim_depth_m)) {
    candidate->rim_depth_m = candidate->centre_rim_depth_m;
  }
  const double along_m = along_m_.load();
  const double heading_rad = heading_rad_.load();
  const bool lay = face_approach_.observe(
      candidate, along_m, heading_rad,
      geometry_.focal_px / static_cast<double>(returns_config_.pixel_stride),
      height_m_.load(), reach_m, smoke_face_config_);
  // Every frame's candidate, for replaying the confirmation offline.
  RCLCPP_INFO_EXPRESSION(
      get_logger(), smoke_face_,
      "PLUME_CANDIDATE along=%.3f heading=%.4f quartile=%.2f reach=%.0f lay=%d %s",
      along_m, heading_rad, quartile_m, reach_m, lay ? 1 : 0,
      candidate != nullptr ? silentPatchSummary(*candidate).c_str() : "none");
  if (lay) {
    candidate->closed = true;
    candidate->reason = "plume";
  }
  std::vector<const SilentPatch*> faces;
  double face_range_m = std::numeric_limits<double>::quiet_NaN();
  const SilentPatch* largest_region{nullptr};
  for (const SilentPatch& region : lit_regions) {
    if (largest_region == nullptr || region.cells > largest_region->cells) {
      largest_region = &region;
    }
    if (region.closed) {
      faces.push_back(&region);
      face_range_m = std::isfinite(face_range_m)
                         ? std::min(face_range_m, region.rim_depth_m)
                         : region.rim_depth_m;
    }
  }
  const std::string largest_text =
      largest_region == nullptr ? "none" : silentPatchSummary(*largest_region);
  face_frames_ = faces.empty() ? 0U : face_frames_ + 1U;
  double face_ahead_m = smokeFacesAheadM(
      lit_regions, face_approach_.confirmed(), face_frames_ >= kFaceFrames, geometry_,
      returns_config_.pixel_stride, image_width_ / returns_config_.pixel_stride,
      image_height_ / returns_config_.pixel_stride, kSilentConeHalfAngleRad);
  const double held_m = face_approach_.boundM(along_m_.load());
  if (std::isfinite(held_m) && !(face_ahead_m <= held_m)) {
    face_ahead_m = held_m;
  }
  if (std::isfinite(face_ahead_m)) {
    silent_range_m = std::isfinite(silent_range_m)
                         ? std::min(silent_range_m, face_ahead_m)
                         : face_ahead_m;
    silent_message.data = silent_range_m;
    silent_range_pub_->publish(silent_message);
  }
  // A face's chance far matches are the smoke's texture (K44).
  face_rays_dropped_ += dropEnclosedFreeRays(
      returns, lit_regions, geometry_, returns_config_.pixel_stride, image_width_,
      image_height_, smoke_face_config_.carve_share);
  if (!rectifier_->ideal()) {
    for (StereoDepthReturn& stereo_return : returns) {
      stereo_return.point = toPhysicalFlu(stereo_return.point);
    }
  }
  std::size_t tof_rays{0U};
  // The ring's zones returning smoke (K38).
  std::size_t tof_zones{0U};
  std::size_t smoky_zones{0U};
  // Item 29 (K12): the ring's surfaces in the camera's field the frame shows dark.
  std::size_t surfaces_in_view{0U};
  std::size_t unlit_surfaces{0U};
  const double gain = camera_gain_.load(std::memory_order_relaxed);
  for (const TofSensor& sensor : tof_sensors_) {
    // The scan nearest the pair's moment (a pair takes longer to match than a scan to
    // arrive).
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan =
        stereo_depth_node_internal::nearestScan(sensor.scans,
                                                rclcpp::Time{left->header.stamp},
                                                kTofStampToleranceS, tof_mutex_);
    if (scan == nullptr) {
      continue;
    }
    const std::optional<std::vector<Point3>> zones = decodePointCloudReturns(*scan);
    if (!zones.has_value()) {
      continue;
    }
    const std::vector<float> intensities =
        decodePointCloudFloatField(*scan, "intensity").value_or(std::vector<float>{});
    std::vector<StereoDepthReturn> rays =
        tofZoneReturns(*zones, sensor.config, intensities);
    if (&sensor == &tof_sensors_[kFrontTofSlot]) {
      std::erase_if(rays, [&](const StereoDepthReturn& ray) {
        if (ray.hit) {
          return false;
        }
        const Eigen::Vector3d rectified =
            rectified_to_physical_flu_.transpose() *
            Eigen::Vector3d{ray.point.x, ray.point.y, ray.point.z};
        const std::optional<std::size_t> cell =
            strideCellOf(Point3{rectified.x(), rectified.y(), rectified.z()}, geometry_,
                         returns_config_.pixel_stride, image_width_, image_height_);
        return cell.has_value() && seen_cells[*cell] == 0U &&
               static_cast<double>(raw_cells[*cell]) >= smoke_face_config_.lit_raw_grey;
      });
    }
    tof_rays += rays.size();
    tof_zones += zones->size();
    for (std::size_t zone = 0U;
         zone < zones->size() && intensities.size() == zones->size(); ++zone) {
      smoky_zones += intensities[zone] > 0.0F &&
                             intensities[zone] < kSmokeIntensityShare &&
                             std::isfinite((*zones)[zone].x)
                         ? 1U
                         : 0U;
    }
    for (const StereoDepthReturn& ray : rays) {
      if (!ray.hit || ray.unobservable) {
        continue;
      }
      const Eigen::Vector3d rectified =
          rectified_to_physical_flu_.transpose() *
          Eigen::Vector3d{ray.point.x, ray.point.y, ray.point.z};
      if (rectified.x() <= 0.0) {
        continue;
      }
      const double column =
          geometry_.principal_x_px - geometry_.focal_px * rectified.y() / rectified.x();
      const double row =
          geometry_.principal_y_px - geometry_.focal_px * rectified.z() / rectified.x();
      const cv::Rect window{static_cast<int>(std::lround(column)) -
                                kSurfaceWindowPx / 2,
                            static_cast<int>(std::lround(row)) - kSurfaceWindowPx / 2,
                            kSurfaceWindowPx, kSurfaceWindowPx};
      if ((window & cv::Rect{0, 0, left_grey.cols, left_grey.rows}) != window) {
        continue;
      }
      ++surfaces_in_view;
      unlit_surfaces += cv::mean(left_grey(window))[0] / gain < kUnlitRawGrey ? 1U : 0U;
    }
    returns.insert(returns.end(), rays.begin(), rays.end());
  }
  // The heat source (K49): its range from the hits in its cone, else the
  // conservative one; the returns inside its standoff dropped; the speed's
  // boundary before the standoff (0 stops, K44's law).
  const double pair_stamp_s = rclcpp::Time{left->header.stamp}.seconds();
  const std::optional<ThermalReading> heat = heatReading(pair_stamp_s);
  double heat_range_m = std::numeric_limits<double>::quiet_NaN();
  bool heat_ranged{false};
  if (heat.has_value()) {
    const std::optional<Point3> fix = heat_fix_snapshot_.point();
    Eigen::Vector3d origin;
    Eigen::Vector3d bearing;
    {
      const std::lock_guard<std::mutex> lock{pose_mutex_};
      origin = body_position_;
      bearing =
          body_orientation_ *
          Eigen::Vector3d{heat->direction.x, heat->direction.y, heat->direction.z};
    }
    heat_range_m = stereo_depth_node_internal::heatRange(
        *heat, returns, kHeatConeMarginRad,
        fix.has_value()
            ? std::optional{(Eigen::Vector3d{fix->x, fix->y, fix->z} - origin).norm()}
            : std::nullopt,
        thermal_config_.range_by_size_error_share, heat_size_range_m_, heat_error_m_);
    heat_point_m_ = fix.has_value() ? Eigen::Vector3d{fix->x, fix->y, fix->z}
                                    : Eigen::Vector3d{origin + heat_range_m * bearing};
    // Seen but not ranged (no hits, fix or size yet, r2289): no bound, no mark.
    heat_ranged = std::isfinite(heat_range_m);
  }
  if (heat_ranged) {
    const double boundary_m =
        std::max(0.0, heat_range_m - heat_error_m_ - kHeatStandoffM);
    silent_range_m = std::isfinite(silent_range_m)
                         ? std::min(silent_range_m, boundary_m)
                         : boundary_m;
    silent_message.data = silent_range_m;
    silent_range_pub_->publish(silent_message);
    // The source in the map (the fix, else the range point): the evidence's point.
    const Eigen::Vector3d at = heat_point_m_;
    geometry_msgs::msg::PointStamped point;
    point.header.stamp = left->header.stamp;
    point.header.frame_id = "map";
    point.point.x = at.x();
    point.point.y = at.y();
    point.point.z = at.z();
    heat_point_pub_->publish(point);
  }
  if (heat.has_value() != heat_seen_) {
    heat_seen_ = heat.has_value();
    std_msgs::msg::String event;
    event.data = stereo_depth_node_internal::heatEventText(heat, heat_range_m);
    mission_event_pub_->publish(event);
    RCLCPP_WARN(get_logger(), "%s", event.data.c_str());
  }
  std_msgs::msg::Float64 smoke_message;
  smoke_message.data =
      tof_zones > 0U ? static_cast<double>(smoky_zones) / static_cast<double>(tof_zones)
                     : 0.0;
  smoke_share_pub_->publish(smoke_message);
  std_msgs::msg::Float64 unlit_message;
  unlit_message.data = surfaces_in_view > 0U ? static_cast<double>(unlit_surfaces) /
                                                   static_cast<double>(surfaces_in_view)
                                             : std::numeric_limits<double>::quiet_NaN();
  unlit_surface_pub_->publish(unlit_message);
  // The frame's observed share as the braking contract reads it.
  const std::size_t frame_samples = (image_width_ / returns_config_.pixel_stride) *
                                    (image_height_ / returns_config_.pixel_stride);
  const double observed_share =
      static_cast<double>(returns.size()) / static_cast<double>(frame_samples);
  // The void's witness (K46): the pair's matches within the confident depth.
  const double confident_depth_m = stereoConfidentDepthM(geometry_, returns_config_);
  const double matched_within_reach_share =
      static_cast<double>(
          std::count_if(returns.begin(),
                        returns.begin() + static_cast<std::ptrdiff_t>(stereo_returns),
                        [confident_depth_m](const StereoDepthReturn& ray) {
                          return ray.hit && ray.point.x <= confident_depth_m;
                        })) /
      static_cast<double>(frame_samples);
  // A blind frame's stereo returns never reach the memory (r1175), the ring's do.
  const bool blind =
      sensorFrameShare(observed_share, headroom) < kSensorBlindFrameShare;
  if (blind) {
    returns.erase(returns.begin(),
                  returns.begin() + static_cast<std::ptrdiff_t>(stereo_returns));
  }
  // Under a sound light the dark is the place's: nothing marked (item 29, r1473).
  const double light_reading = light_reading_.load(std::memory_order_relaxed);
  const bool light_sound =
      std::isfinite(light_reading) &&
      light_reading >= LightReliabilityJudgment::kLightReadingOutage;
  const double stamp_s = rclcpp::Time{left->header.stamp}.seconds();
  std::vector<Point3> unobservable =
      unobservableFrustum(stamp_s, blind && !light_sound);
  const bool void_frame =
      isVoidFrame({.light_sound = light_sound,
                   .raw_brightness_p95 = raw_brightness,
                   .matched_within_reach_share = matched_within_reach_share,
                   .ring_surfaces_in_view = surfaces_in_view});
  std_msgs::msg::Float64 void_message;
  void_message.data =
      void_frame ? confident_depth_m : std::numeric_limits<double>::quiet_NaN();
  void_range_pub_->publish(void_message);
  if (stamp_s - face_emitted_s_ >= kFaceEmitPeriodS || stamp_s < face_emitted_s_) {
    const std::size_t pitch_cells = std::max<std::size_t>(
        1U, static_cast<std::size_t>(kRayPitchPx) / returns_config_.pixel_stride);
    std::vector<Point3> rays;
    if (face_approach_.confirmed()) {
      rays =
          smokeFaceRays(lit_regions, "plume", geometry_, returns_config_.pixel_stride,
                        image_width_, kNearestM, smoke_face_config_.face_slab_m, kStepM,
                        pitch_cells, smoke_face_config_.halo_m);
    }
    if (face_frames_ >= kFaceFrames) {
      const std::vector<Point3> body = smokeFaceRays(
          lit_regions, "body", geometry_, returns_config_.pixel_stride, image_width_,
          kNearestM, smoke_face_config_.face_slab_m, kStepM, pitch_cells);
      rays.insert(rays.end(), body.begin(), body.end());
    }
    if (!rays.empty()) {
      face_emitted_s_ = stamp_s;
      unobservable.insert(unobservable.end(), rays.begin(), rays.end());
      face_marks_total_ += rays.size();
      const std::string laid =
          (face_approach_.confirmed() ? smokeFacesLaid(lit_regions, "plume", kNearestM)
                                      : "") +
          (face_frames_ >= kFaceFrames ? smokeFacesLaid(lit_regions, "body", kNearestM)
                                       : "");
      RCLCPP_WARN(get_logger(), "SMOKE_FACE_MARKS rays=%zu%s", rays.size(),
                  laid.c_str());
    }
  }
  if (!rectifier_->ideal()) {
    for (Point3& point : unobservable) {
      point = toPhysicalFlu(point);
    }
  }
  if (heat_ranged &&
      (stamp_s - heat_emitted_s_ >= kFaceEmitPeriodS || stamp_s < heat_emitted_s_)) {
    const std::vector<Point3> cap = heatStandoffCap(
        *heat, heat_range_m, kHeatStandoffM + heat_error_m_, kHeatMarkSpacingM);
    heat_emitted_s_ = stamp_s;
    unobservable.insert(unobservable.end(), cap.begin(), cap.end());
    heat_marks_total_ += cap.size();
    RCLCPP_WARN(get_logger(), "%s",
                stereo_depth_node_internal::heatMarksText(cap.size(), heat_range_m,
                                                          *heat, heat_fix_snapshot_)
                    .c_str());
  }
  if (heat_ranged) {
    // The returns through the standoff ball (the plume's texture, the
    // surfaces beyond, K49) dropped after the frame's counts, which index
    // the pair's returns at the front (r2235: dropped before, the process died).
    std::erase_if(returns, [&](const StereoDepthReturn& ray) {
      return !ray.unobservable && crossesHeatStandoff(ray.point, *heat, heat_range_m,
                                                      kHeatStandoffM + heat_error_m_);
    });
  }
  unobservable_total_ += unobservable.size();
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = left->header.stamp;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2Modifier modifier{cloud};
  // intensity: 1 a surface, 0 a free ray, 2 looked at, not seen (item 17 stage 8).
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y",
                                1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(returns.size() + unobservable.size());
  sensor_msgs::PointCloud2Iterator<float> x{cloud, "x"};
  sensor_msgs::PointCloud2Iterator<float> y{cloud, "y"};
  sensor_msgs::PointCloud2Iterator<float> z{cloud, "z"};
  sensor_msgs::PointCloud2Iterator<float> flag{cloud, "intensity"};
  std::size_t hits{0U};
  for (const StereoDepthReturn& ray : returns) {
    *x = static_cast<float>(ray.point.x);
    *y = static_cast<float>(ray.point.y);
    *z = static_cast<float>(ray.point.z);
    *flag = ray.unobservable ? 2.0F : static_cast<float>(ray.hit);
    hits += ray.hit ? 1U : 0U;
    ++x;
    ++y;
    ++z;
    ++flag;
  }
  for (const Point3& point : unobservable) {
    *x = static_cast<float>(point.x);
    *y = static_cast<float>(point.y);
    *z = static_cast<float>(point.z);
    *flag = 2.0F;
    ++x;
    ++y;
    ++z;
    ++flag;
  }
  returns_pub_->publish(cloud);
  ++pairs_;
  const double match_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - started)
                              .count();
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "STEREO_DEPTH pairs=%zu hits=%zu free_rays=%zu tof_rays=%zu "
      "match_ms=%.1f brightness_by_metre=%s headroom=%.3f "
      "lit_range_m=%.1f light_share=%.3f raw_brightness=%.1f "
      "observed_share=%.3f blind_dropped=%zu noise=%.1f "
      "unobservable_total=%zu tof_in_view=%zu tof_unlit=%zu "
      "silent_patches=%zu closed=%zu silent_range_m=%.1f "
      "tof_smoky_zones=%zu smoke_faces=%zu face_range_m=%.1f "
      "face_marks_total=%zu free_dropped=%zu largest_lit=%s void=%s "
      "within_reach=%.3f heat=%.4f heat_range_m=%.1f heat_marks_total=%zu",
      pairs_, hits, returns.size() - hits, tof_rays, match_ms,
      stereo_depth_node_internal::brightnessByMetreText(brightness_by_metre).c_str(),
      headroom, lit_range_m, light_share, raw_brightness, observed_share,
      blind ? stereo_returns : 0U, noise, unobservable_total_, surfaces_in_view,
      unlit_surfaces, patches.size(), closed_patches, silent_range_m, smoky_zones,
      faces.size(), face_range_m, face_marks_total_, face_rays_dropped_,
      largest_text.c_str(), void_frame ? "true" : "false", matched_within_reach_share,
      heat.has_value() ? heat->hot_share : 0.0, heat_range_m, heat_marks_total_);
}

} // namespace drone_city_nav
