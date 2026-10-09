// PathConverter's height-map lookup (path_converter_node). The class lives
// in src/path_converter_node.cpp, next to main(), with no header, so this
// test compiles that file with main() renamed. The source is unchanged.
//
// M29_* guards the fix of audit M-29 (an x outside the row wrapped into the
// next row).

#include <gtest/gtest.h>

// Everything path_converter_node.cpp includes, before main is renamed.
#include <geometry_msgs/msg/pose.hpp>
#include <mapconversion_msgs/msg/height_map.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "datatypes.hh"

#define main path_converter_node_main
#include "../src/path_converter_node.cpp"
#undef main

#include <cmath>
#include <memory>

namespace {

class PathConverterHeights : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<PathConverter>();
    // 3 x 2 cells, row-major: index = x + y * width.
    mapconversion_msgs::msg::HeightMap map;
    map.info.resolution = 0.5f;
    map.info.width = 3;
    map.info.height = 2;
    map.bottom = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    map.top = {10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f};
    node_->heightCallback(map);
  }

  std::shared_ptr<PathConverter> node_;
};

TEST_F(PathConverterHeights, CellsInsideTheMapReadTheirRowMajorEntry) {
  EXPECT_DOUBLE_EQ(node_->getHeight(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(node_->getHeight(2, 0), 2.0);
  EXPECT_DOUBLE_EQ(node_->getHeight(0, 1), 3.0);
  EXPECT_DOUBLE_EQ(node_->getHeight(2, 1), 5.0);
  EXPECT_DOUBLE_EQ(node_->getHeightTop(1, 1), 14.0);
}

TEST_F(PathConverterHeights, CellsPastTheFirstOrLastEntryAreUnknown) {
  EXPECT_TRUE(std::isnan(node_->getHeight(0, 2)));
  EXPECT_TRUE(std::isnan(node_->getHeight(-1, 0)));
  EXPECT_TRUE(std::isnan(node_->getHeightTop(0, -1)));
  EXPECT_TRUE(std::isnan(node_->getHeightTop(3, 1)));
}

// M-29: only the flat index was checked, so an x outside [0, width) wrapped
// into the neighbouring row and read another cell's height.
TEST_F(PathConverterHeights, M29_OutOfRangeXIsUnknown) {
  EXPECT_TRUE(std::isnan(node_->getHeight(3, 0)));      // reads (0, 1)
  EXPECT_TRUE(std::isnan(node_->getHeight(-1, 1)));     // reads (2, 0)
  EXPECT_TRUE(std::isnan(node_->getHeightTop(3, 0)));   // reads (0, 1)
  EXPECT_TRUE(std::isnan(node_->getHeightTop(-1, 1)));  // reads (2, 0)
}

}  // namespace
