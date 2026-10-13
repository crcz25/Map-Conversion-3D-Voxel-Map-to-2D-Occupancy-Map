// MapToMap's visualization maps (map_conversion_oct_node). The class lives
// in src/map_conversion_oct_node.cpp, next to main(), with no header, so this
// test compiles that file with main() renamed. The source is unchanged.
//
// This test is built with -fsanitize=float-divide-by-zero,float-cast-overflow
// (not recoverable): a NaN or out-of-range double written to an int8 cell,
// or a division by a zero height range, stops it (M-27).

#include <gtest/gtest.h>

// Everything map_conversion_oct_node.cpp includes, before main is renamed.
#include <cstddef>
#include <cstdio>
#include <memory>
#include <vector>

#include <mapconversion_msgs/msg/height_map.hpp>
#include <mapconversion_msgs/msg/slope_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rmw/types.h>

#include "MapConverter.hh"
#include "octomap/octomap_types.h"
#include "octomap_msgs/msg/octomap.hpp"

#define main map_conversion_oct_node_main
#include "../src/map_conversion_oct_node.cpp"  // NOLINT(bugprone-suspicious-include): the test seam
#undef main

#include <map>
#include <string>
#include <vector>

#include <std_msgs/msg/header.hpp>

namespace {

using nav_msgs::msg::OccupancyGrid;

// A 2 m x 2 m floor at one height (occupied at z = 0.05, free up to
// z = 1.95), 0.1 m voxels: every known cell has the same floor height, and
// the unevaluated border of the grid stays unknown (NaN).
octomap_msgs::msg::Octomap flatFloor() {
  octomap::OcTree tree(0.1);
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {
      const float x = 0.05f + 0.1f * static_cast<float>(i);
      const float y = 0.05f + 0.1f * static_cast<float>(j);
      tree.updateNode(octomap::point3d(x, y, 0.05f), true);
      for (int k = 1; k < 20; ++k) {
        tree.updateNode(octomap::point3d(x, y, 0.05f + 0.1f * static_cast<float>(k)),
                        false);
      }
    }
  }
  octomap_msgs::msg::Octomap msg;
  msg.header.frame_id = "map";
  octomap_msgs::fullMapToMsg(tree, msg);
  return msg;
}

class MapToMapVisualization : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<MapToMap>();
    listener_ = std::make_shared<rclcpp::Node>("visualization_listener");
    for (const char *topic : {"/visualization_floor_map", "/visualization_ceiling_map",
                              "/visualization_slope_map"}) {
      subs_.push_back(listener_->create_subscription<OccupancyGrid>(
          topic, 5, [this, topic](OccupancyGrid::SharedPtr m) {
            received_[topic] = std::move(m);
          }));
    }
    executor_.add_node(listener_);
    // Bounded wait for discovery: pub() only fills maps that have readers.
    for (int i = 0; i < 500; ++i) {
      bool matched = true;
      for (const auto &s : subs_) {
        matched = matched && s->get_publisher_count() > 0;
      }
      if (matched) {
        break;
      }
      executor_.spin_some(std::chrono::milliseconds(10));
    }
  }

  // The maps published for `msg`.
  void convert(const octomap_msgs::msg::Octomap &msg) {
    node_->mapCallback(msg);  // updates the 2D map and calls pub()
    for (int i = 0; i < 500 && received_.size() < 3; ++i) {
      executor_.spin_some(std::chrono::milliseconds(10));
    }
  }

  std::shared_ptr<MapToMap> node_;
  std::shared_ptr<rclcpp::Node> listener_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<rclcpp::Subscription<OccupancyGrid>::SharedPtr> subs_;
  std::map<std::string, OccupancyGrid::SharedPtr> received_;
};

// M-27: unknown cells (NaN height or slope) are written as -1, and a map of
// one height (zero range) is not divided by zero.
TEST_F(MapToMapVisualization, UnknownCellsAreMinusOneAndAFlatMapIsNotDividedByZero) {
  convert(flatFloor());
  ASSERT_EQ(received_.size(), 3u);
  for (const auto &[topic, map] : received_) {
    ASSERT_FALSE(map->data.empty()) << topic;
    size_t unknown = 0, known = 0;
    for (const int8_t v : map->data) {
      unknown += v == -1;
      known += v != -1;
    }
    EXPECT_GT(unknown, 0u) << topic;  // the unevaluated border
    EXPECT_GT(known, 0u) << topic;
  }
  // One floor height: every known floor cell is the bottom of the colour
  // range (0).
  for (const int8_t v : received_.at("/visualization_floor_map")->data) {
    EXPECT_TRUE(v == -1 || v == 0) << static_cast<int>(v);
  }
}

// M-28: every map output carries the OctoMap's stamp and frame; a
// non-empty map_frame parameter overrides the frame only.
class MapToMapHeaders : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  template <typename Msg> void listen(const std::string &topic) {
    subs_.push_back(listener_->create_subscription<Msg>(
        topic, 5, [this, topic](typename Msg::SharedPtr m) {
          headers_[topic] = m->header;
        }));
  }

  // Converts flatFloor() stamped (1234 s, 5678 ns) in "octomap_frame" with
  // a node built from `options`; returns false if not every output arrived.
  bool convert(const rclcpp::NodeOptions &options) {
    node_ = std::make_shared<MapToMap>(options);
    listener_ = std::make_shared<rclcpp::Node>("header_listener");
    listen<OccupancyGrid>("/mapUAV");
    listen<OccupancyGrid>("/mapUGV");
    listen<mapconversion_msgs::msg::HeightMap>("/heightMap");
    listen<mapconversion_msgs::msg::SlopeMap>("/slopeMap");
    listen<OccupancyGrid>("/visualization_floor_map");
    listen<OccupancyGrid>("/visualization_ceiling_map");
    listen<OccupancyGrid>("/visualization_slope_map");
    executor_.add_node(listener_);
    // Bounded wait for discovery: pub() only fills maps that have readers.
    for (int i = 0; i < 500; ++i) {
      bool matched = true;
      for (const auto &sub : subs_) {
        matched = matched && sub->get_publisher_count() > 0;
      }
      if (matched) {
        break;
      }
      executor_.spin_some(std::chrono::milliseconds(10));
    }
    octomap_msgs::msg::Octomap msg = flatFloor();
    msg.header.stamp.sec = 1234;
    msg.header.stamp.nanosec = 5678;
    msg.header.frame_id = "octomap_frame";
    node_->mapCallback(msg);
    for (int i = 0; i < 500 && headers_.size() < subs_.size(); ++i) {
      executor_.spin_some(std::chrono::milliseconds(10));
    }
    return headers_.size() == subs_.size();
  }

  void expectHeaders(const std::string &frame) {
    for (const auto &[topic, header] : headers_) {
      EXPECT_EQ(header.stamp.sec, 1234) << topic;
      EXPECT_EQ(header.stamp.nanosec, 5678u) << topic;
      EXPECT_EQ(header.frame_id, frame) << topic;
    }
  }

  std::shared_ptr<MapToMap> node_;
  std::shared_ptr<rclcpp::Node> listener_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::map<std::string, std_msgs::msg::Header> headers_;
};

TEST_F(MapToMapHeaders, M28_OutputsCarryTheInputStampAndFrame) {
  ASSERT_TRUE(convert(rclcpp::NodeOptions())) << headers_.size() << " outputs";
  expectHeaders("octomap_frame");
}

TEST_F(MapToMapHeaders, M28_MapFrameOverridesOnlyTheFrame) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"map_frame", "override_frame"}});
  ASSERT_TRUE(convert(options)) << headers_.size() << " outputs";
  expectHeaders("override_frame");
}

}  // namespace
