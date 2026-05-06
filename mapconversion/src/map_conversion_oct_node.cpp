#include "MapConverter.hh"
#include "octomap/octomap_types.h"
#include "octomap_msgs/msg/octomap.hpp"
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <mapconversion_msgs/msg/height_map.hpp>
#include <mapconversion_msgs/msg/slope_map.hpp>
#include <memory>
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
#include <vector>

using namespace std;

class MapToMap : public rclcpp::Node {
private:
  // subs
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subOctMap;

  // pub
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubMapUGV,
      pubMapUAV, pubMapFloor, pubMapCeiling, pubMapSlopeVis;
  rclcpp::Publisher<mapconversion_msgs::msg::HeightMap>::SharedPtr pubHeightMap;
  rclcpp::Publisher<mapconversion_msgs::msg::SlopeMap>::SharedPtr pubMapSlope;
  // msg
  nav_msgs::msg::OccupancyGrid mapMsg;
  mapconversion_msgs::msg::HeightMap heightMsg;
  mapconversion_msgs::msg::SlopeMap slopeMsg;

  std::unique_ptr<MapConverter> MC;
  std::unique_ptr<octomap::OcTree> OcMap;

  double resolution;
  double slopeMax;
  int slopeEstimationSize;
  double minimumZ;
  int minimumOccupancy;
  string mapFrame;
  double mapZpos;
  bool partial_map_updates;
  bool sub_qos_reliable;
  bool sub_qos_transient_local;
  bool pub_qos_reliable;
  bool pub_qos_transient_local;

public:
  MapToMap() : Node("map_conversion") {
    slopeMax = this->declare_parameter("max_slope_ugv", INFINITY);
    slopeEstimationSize = this->declare_parameter("slope_estimation_size", 1);
    slopeEstimationSize = max(slopeEstimationSize, 2);
    minimumZ = this->declare_parameter("minimum_z", 1.0);
    minimumOccupancy = this->declare_parameter("minimum_occupancy", 10);
    mapFrame = this->declare_parameter("map_frame", string("map"));
    mapZpos = this->declare_parameter("map_position_z", 0.0);
    partial_map_updates = this->declare_parameter("partial_map_updates", false);
    sub_qos_reliable = this->declare_parameter("subscriber_qos_reliable", true);
    pub_qos_reliable = this->declare_parameter("publisher_qos_reliable", true);
    sub_qos_transient_local =
        this->declare_parameter("subscriber_qos_transient_local", false);
    pub_qos_transient_local =
        this->declare_parameter("publisher_qos_transient_local", false);

    // QoS profiles
    rclcpp::QoS sub_qos_profile = rclcpp::QoS(rclcpp::KeepLast(5));
    if (sub_qos_reliable)
      sub_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    else
      sub_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    if (sub_qos_transient_local)
      sub_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    else
      sub_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    rclcpp::QoS pub_qos_profile = rclcpp::QoS(rclcpp::KeepLast(5));
    if (pub_qos_reliable)
      pub_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    else
      pub_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    if (pub_qos_transient_local)
      pub_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    else
      pub_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // Subscribers and publishers
    subOctMap = this->create_subscription<octomap_msgs::msg::Octomap>(
        "octomap", sub_qos_profile,
        std::bind(&MapToMap::mapCallback, this, std::placeholders::_1));
    pubMapUGV = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/mapUGV", pub_qos_profile);
    pubMapUAV = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/mapUAV", pub_qos_profile);
    pubMapFloor = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/visualization_floor_map", 5);
    pubMapCeiling = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/visualization_ceiling_map", 5);
    pubHeightMap = this->create_publisher<mapconversion_msgs::msg::HeightMap>(
        "/heightMap", pub_qos_profile);
    pubMapSlopeVis = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/visualization_slope_map", 5);
    pubMapSlope = this->create_publisher<mapconversion_msgs::msg::SlopeMap>(
        "/slopeMap", pub_qos_profile);
  }

  ~MapToMap() = default;

  void mapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    if (!std::isfinite(msg->resolution) || msg->resolution <= 0.0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Received octomap with invalid resolution %.6f",
                   msg->resolution);
      return;
    }

    if (MC == nullptr) {
      resolution = msg->resolution;
      MC = std::unique_ptr<MapConverter>(new MapConverter(
          resolution, slopeEstimationSize, minimumZ, minimumOccupancy));
    } else if (std::abs(msg->resolution - resolution) > 1e-9) {
      RCLCPP_WARN(this->get_logger(),
                  "Octomap resolution changed from %.6f to %.6f; rebuilding "
                  "the converted map",
                  resolution, msg->resolution);
      resolution = msg->resolution;
      MC = std::unique_ptr<MapConverter>(new MapConverter(
          resolution, slopeEstimationSize, minimumZ, minimumOccupancy));
      OcMap.reset();
    }

    // Convert ROS message to Octomap
    std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::msgToMap(*msg));
    if (tree == nullptr) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to convert octomap message: msgToMap returned NULL");
      return;
    }
    octomap::OcTree *newOcMap = dynamic_cast<octomap::OcTree *>(tree.get());
    if (!(newOcMap)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to cast converted octomap type '%s' to OcTree",
                   msg->id.c_str());
      return;
    }

    // Check if tree is empty
    if (newOcMap->size() == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Received empty octomap, skipping processing");
      return;
    }

    double min_x, min_y, min_z;
    double max_x, max_y, max_z;

    newOcMap->getMetricMin(min_x, min_y, min_z);
    newOcMap->getMetricMax(max_x, max_y, max_z);

    // Validate min/max values (check for NaN and invalid ranges)
    if (std::isnan(min_x) || std::isnan(min_y) || std::isnan(min_z) ||
        std::isnan(max_x) || std::isnan(max_y) || std::isnan(max_z)) {
      RCLCPP_ERROR(this->get_logger(), "Octomap contains NaN values in bounds");
      return;
    }

    if (min_x > max_x || min_y > max_y || min_z > max_z) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid octomap bounds: min > max (x: %.2f > %.2f, y: %.2f > %.2f, z: %.2f > %.2f)",
                   min_x, max_x, min_y, max_y, min_z, max_z);
      return;
    }

    vector<double> minMax(6);
    bool has_update_region = true;
    if (OcMap == nullptr || !partial_map_updates) {
      minMax[0] = min_x;
      minMax[1] = max_x;
      minMax[2] = min_y;
      minMax[3] = max_y;
      OcMap.reset(newOcMap);
      tree.release();
    } else {
      has_update_region = computeBoundingBox(minMax, newOcMap, OcMap.get());
      OcMap.reset(newOcMap);
      tree.release();
    }
    minMax[4] = min_z;
    minMax[5] = max_z;

    if (has_update_region)
      update2Dmap(minMax);
    pub();
  }

  bool computeBoundingBox(vector<double> &minMax, octomap::OcTree *tree1,
                          octomap::OcTree *tree2) {
    // Variables to hold min and max coordinates
    double min_x = INFINITY;
    double min_y = INFINITY;
    double min_z = INFINITY;
    double max_x = -INFINITY;
    double max_y = -INFINITY;
    double max_z = -INFINITY;
    bool changed = false;

    // Iterate over all new leaf nodes and find added or occupancy-changed cells.
    for (octomap::OcTree::leaf_iterator it = tree1->begin_leafs(),
                                        end = tree1->end_leafs();
         it != end; ++it) {
      octomap::OcTreeKey key = it.getKey();
      octomap::OcTreeNode *node2 = tree2->search(key);
      if (node2 != nullptr) {
        bool occupied1 = tree1->isNodeOccupied(*it);
        bool occupied2 = tree2->isNodeOccupied(node2);
        if (occupied1 != occupied2) {
          // Node occupancy has changed
          double x = it.getX();
          double y = it.getY();
          double z = it.getZ();
          updateBoundingBox(x, y, z, min_x, min_y, min_z, max_x, max_y, max_z);
          changed = true;
        }
      } else {
        // Node not found in tree2, so it is new.
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();
        updateBoundingBox(x, y, z, min_x, min_y, min_z, max_x, max_y, max_z);
        changed = true;
      }
    }

    // Iterate over old leaf nodes to find cells removed from the new map.
    for (octomap::OcTree::leaf_iterator it = tree2->begin_leafs(),
                                        end = tree2->end_leafs();
         it != end; ++it) {
      octomap::OcTreeKey key = it.getKey();
      if (tree1->search(key) != nullptr)
        continue;

      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();
      updateBoundingBox(x, y, z, min_x, min_y, min_z, max_x, max_y, max_z);
      changed = true;
    }

    if (!changed)
      return false;

    minMax[0] = min_x;
    minMax[1] = max_x;
    minMax[2] = min_y;
    minMax[3] = max_y;
    minMax[4] = min_z;
    minMax[5] = max_z;
    return true;
  }

  void updateBoundingBox(double x, double y, double z, double &min_x,
                         double &min_y, double &min_z, double &max_x,
                         double &max_y, double &max_z) {
    if (x < min_x)
      min_x = x;
    if (y < min_y)
      min_y = y;
    if (z < min_z)
      min_z = z;
    if (x > max_x)
      max_x = x;
    if (y > max_y)
      max_y = y;
    if (z > max_z)
      max_z = z;
  }

  // Update 2D map in the region of the aabb
  void update2Dmap(vector<double> minMax) {
    // Validate inputs
    if (OcMap == nullptr || MC == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "OcMap or MC is NULL in update2Dmap");
      return;
    }

    // Check for infinity and NaN values
    for (size_t i = 0; i < minMax.size(); ++i) {
      if (std::isinf(minMax[i]) || std::isnan(minMax[i])) {
        RCLCPP_DEBUG(this->get_logger(),
                     "Invalid minMax value at index %zu: %.2f (inf=%d, nan=%d)",
                     i, minMax[i], std::isinf(minMax[i]), std::isnan(minMax[i]));
        return;
      }
    }

    vector<voxel> voxelList;
    // get all free and occupide voxels in boundign box
    octomap::point3d minPoint(minMax[0], minMax[2], minMax[4]);
    octomap::point3d maxPoint(minMax[1], minMax[3], minMax[5]);

    // Validate bounding box points
    if (minPoint.x() > maxPoint.x() || minPoint.y() > maxPoint.y() || minPoint.z() > maxPoint.z()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid bounding box: min point (%.2f, %.2f, %.2f) > max point (%.2f, %.2f, %.2f)",
                   minPoint.x(), minPoint.y(), minPoint.z(),
                   maxPoint.x(), maxPoint.y(), maxPoint.z());
      return;
    }

    for (auto it = OcMap->begin_leafs_bbx(minPoint, maxPoint),
              it_end = OcMap->end_leafs_bbx();
         it != it_end; ++it) {
      voxel v;
      v.position.x = it.getX();
      v.position.y = it.getY();
      v.position.z = it.getZ();
      v.halfSize = it.getSize() / 2;
      v.occupied = OcMap->isNodeOccupied(*it);
      voxelList.push_back(v);
    }
    MC->updateMap(voxelList, minMax);
  }

  void pub() {
    // Safety check for null pointers
    if (MC == nullptr || OcMap == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "MC or OcMap is NULL in pub()");
      return;
    }

    mapMsg.header.stamp = this->now();
    mapMsg.header.frame_id = mapFrame;
    heightMsg.header = mapMsg.header;
    const int map_width = MC->map.sizeX();
    const int map_height = MC->map.sizeY();
    const double map_resolution = MC->map.getResulution();
    if (map_width <= 0 || map_height <= 0 || map_resolution <= 0 ||
        !std::isfinite(map_resolution)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid map dimensions: width=%d, height=%d, "
                   "resolution=%.4f",
                   map_width, map_height, map_resolution);
      return;
    }

    const auto width = static_cast<size_t>(map_width);
    const auto height = static_cast<size_t>(map_height);
    if (width > std::numeric_limits<uint32_t>::max() ||
        height > std::numeric_limits<uint32_t>::max() ||
        width > std::numeric_limits<size_t>::max() / height) {
      RCLCPP_ERROR(this->get_logger(),
                   "Map dimensions are too large to publish: width=%d, "
                   "height=%d",
                   map_width, map_height);
      return;
    }

    const size_t cell_count = width * height;
    mapMsg.info.width = static_cast<uint32_t>(map_width);
    mapMsg.info.height = static_cast<uint32_t>(map_height);
    mapMsg.info.resolution = MC->map.getResulution();

    mapMsg.info.origin.position.x = MC->map.offsetX();
    mapMsg.info.origin.position.y = MC->map.offsetY();
    mapMsg.info.origin.position.z = mapZpos;
    mapMsg.info.origin.orientation.x = 0;
    mapMsg.info.origin.orientation.y = 0;
    mapMsg.info.origin.orientation.z = 0;
    mapMsg.info.origin.orientation.w = 1;
    heightMsg.info = mapMsg.info;
    slopeMsg.info = mapMsg.info;
    mapMsg.data.resize(cell_count);
    heightMsg.top.resize(cell_count);
    heightMsg.bottom.resize(cell_count);
    slopeMsg.slope.resize(cell_count);

    // pub map for UGV
    if (pubMapUGV->get_subscription_count() != 0 || pub_qos_transient_local) {
      if (isinf(slopeMax)) {
        RCLCPP_WARN(
            this->get_logger(),
            "The max slope UGV is not set; obstacles will not be included in "
            "the UGV map. \nAdd \'_max_slope_ugv:=x\' to rosrun command, or "
            "\'<param name=\"max_slope_ugv\" value=\"x\"/>\' in launch file.");
      }

      mapMsg.header.stamp = this->now();
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          size_t index = static_cast<size_t>(x) +
                         static_cast<size_t>(y) * width;

          mapMsg.data[index] = MC->map.get(x, y, slopeMax);
        }
      }
      pubMapUGV->publish(mapMsg);
    }

    // pub map for UAV
    if (pubMapUAV->get_subscription_count() != 0 || pub_qos_transient_local) {
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          size_t index = static_cast<size_t>(x) +
                         static_cast<size_t>(y) * width;

          mapMsg.data[index] = MC->map.get(x, y);
        }
      }
      pubMapUAV->publish(mapMsg);
    }

    // pub rviz visualization for floor heigth map
    if (pubMapFloor->get_subscription_count() != 0) {
      if (MC->map.sizeY() == 0 || MC->map.sizeX() == 0)
        return;
      double vMax = NAN, vMin = NAN;
      // find max min for normalization
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          double v = MC->map.getHeight(x, y);
          if (isnan(v))
            continue;
          if (isnan(vMax))
            vMax = v;
          else
            vMax = max(vMax, v);
          if (isnan(vMin))
            vMin = v;
          else
            vMin = min(vMin, v);
        }
      }

      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          size_t index = static_cast<size_t>(x) +
                         static_cast<size_t>(y) * width;
          double value = MC->map.getHeight(x, y);
          if (!isnan(value)) {
            value = (value - vMin) / (vMax - vMin) * 199;
            if (value > 99)
              value -= 199; // To get the full color range in the rviz cost map
          }
          mapMsg.data[index] = value;
        }
      }

      pubMapFloor->publish(mapMsg);
    }
    // pub rviz visualization for cieling heigth map
    if (pubMapCeiling->get_subscription_count() != 0) {
      if (MC->map.sizeY() == 0 || MC->map.sizeX() == 0)
        return;
      double vMax = NAN, vMin = NAN;
      // find max min for normalization
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          double v = MC->map.getHeightTop(x, y);
          if (isnan(v))
            continue;
          if (isnan(vMax))
            vMax = v;
          else
            vMax = max(vMax, v);
          if (isnan(vMin))
            vMin = v;
          else
            vMin = min(vMin, v);
        }
      }

      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          size_t index = static_cast<size_t>(x) +
                         static_cast<size_t>(y) * width;
          double value = MC->map.getHeightTop(x, y);
          if (!isnan(value)) {
            value = (value - vMin) / (vMax - vMin) * 199;
            if (value > 99)
              value -= 199; // To get the full color range in the rviz cost map
          }
          mapMsg.data[index] = value;
        }
      }

      pubMapCeiling->publish(mapMsg);
    }

    // pub height map
    if (pubHeightMap->get_subscription_count() != 0 ||
        pub_qos_transient_local) {
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          size_t index = static_cast<size_t>(x) +
                         static_cast<size_t>(y) * width;

          heightMsg.bottom[index] = MC->map.getHeight(x, y);
          heightMsg.top[index] = MC->map.getHeightTop(x, y);
        }
      }
      pubHeightMap->publish(heightMsg);
    }

    // pub rviz visualization fro slope map
    if (pubMapSlopeVis->get_subscription_count() != 0) {
      double vMax = slopeMax * 2, vMin = 0;

      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          size_t index = static_cast<size_t>(x) +
                         static_cast<size_t>(y) * width;
          double value = min(MC->map.getSlope(x, y), slopeMax * 2);
          if (!isnan(value)) {
            value = (value - vMin) / (vMax - vMin) * 198 + 1;
            if (value > 99)
              value -= 199; // soooo basically, rviz costmap are weird...
          }
          mapMsg.data[index] = value;
        }
      }
      pubMapSlopeVis->publish(mapMsg);
    }

    // pub slope map
    if (pubMapSlope->get_subscription_count() != 0 || pub_qos_transient_local) {
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          size_t index = static_cast<size_t>(x) +
                         static_cast<size_t>(y) * width;

          slopeMsg.slope[index] = MC->map.getSlope(x, y);
        }
      }
      pubMapSlope->publish(slopeMsg);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto Node = make_shared<MapToMap>();
  rclcpp::spin(Node);
  rclcpp::shutdown();

  return 0;
}
