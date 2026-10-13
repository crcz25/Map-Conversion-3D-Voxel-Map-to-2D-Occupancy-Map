// MapConverter::updateMap: voxels to the 2D occupancy and height maps, and
// the occupancy written to unknown neighbours of free cells. The
// H05_* tests cover audit H-05 (the neighbour write used the centre cell's
// y).
//
// The map is a 7 x 7 grid of 1 m cells. Every cell has a 1 m free voxel
// except the wall cell (3, 4), which has an occupied voxel of the same
// height: an unknown cell, fully occupied next to free space.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "MapConverter.hh"

namespace {

constexpr int kSize = 7;
constexpr int kWallX = 3;
constexpr int kWallY = 4;

voxel cellVoxel(int x, int y, bool occupied) {
  // Centred in cell (x, y), from z = 0 to z = 1.
  return voxel{{x + 0.5, y + 0.5, 0.5}, 0.5f, occupied};
}

MapConverter convertedRoom(
    const std::vector<std::pair<int, int>> &walls = {{kWallX, kWallY}}) {
  std::vector<voxel> voxels;
  for (int x = 0; x < kSize; ++x) {
    for (int y = 0; y < kSize; ++y) {
      const bool wall =
          std::find(walls.begin(), walls.end(), std::make_pair(x, y)) !=
          walls.end();
      voxels.push_back(cellVoxel(x, y, wall));
    }
  }
  // resolution 1 m, slope window 1, minimum free height 0.5 m, minimum
  // occupancy 50.
  MapConverter converter(1.0, 1, 0.5, 50);
  converter.updateMap(voxels, {0.0, kSize, 0.0, kSize, 0.0, 1.0});
  return converter;
}

TEST(MapConverter, MinMaxVoxelUsesOnlyTheSmallestVoxels) {
  MapConverter converter(1.0, 1, 0.5, 50);
  const std::vector<voxel> voxels = {
      {{1.0, 2.0, 3.0}, 0.5f, false},
      {{-4.0, 5.0, -6.0}, 0.5f, true},
      {{100.0, 100.0, 100.0}, 2.0f, false},  // larger than the resolution
  };
  EXPECT_EQ(converter.minMaxVoxel(voxels),
            (std::vector<double>{-4.0, 1.0, 2.0, 5.0, -6.0, 3.0}));
}

TEST(MapConverter, EmptyOrUnboundedUpdateLeavesTheMapUntouched) {
  MapConverter converter(1.0, 1, 0.5, 50);
  converter.updateMap({}, {0.0, 1.0, 0.0, 1.0, 0.0, 1.0});
  EXPECT_EQ(converter.map.sizeX(), 0);
  // Only voxels larger than the resolution: no usable bounds.
  const std::vector<double> unbounded =
      converter.minMaxVoxel({{{0.5, 0.5, 0.5}, 2.0f, false}});
  ASSERT_TRUE(std::isinf(unbounded[0]));
  converter.updateMap({cellVoxel(0, 0, false)}, unbounded);
  EXPECT_EQ(converter.map.sizeX(), 0);
}

TEST(MapConverter, FreeCellsAreFreeWithFloorAndCeilingHeights) {
  MapConverter converter = convertedRoom();
  for (const auto &[x, y] : std::vector<std::pair<int, int>>{
           {1, 1}, {2, 2}, {5, 5}, {1, 5}, {5, 1}}) {
    EXPECT_EQ(converter.map.get(x, y), 0) << x << "," << y;
    // Free range [0, 1] enlarged by resolution * 0.001.
    EXPECT_NEAR(converter.map.getHeight(x, y), -0.001, 1e-9);
    EXPECT_NEAR(converter.map.getHeightTop(x, y), 1.001, 1e-9);
  }
  // The border ring is outside the evaluated area.
  EXPECT_EQ(converter.map.get(0, 0), -1);
}

TEST(MapConverter, HorizontalNeighbourWriteMarksTheWallOccupied) {
  // Free cells (2, 4) and (4, 4) reach the wall along x; that write uses the
  // wall's own coordinates.
  MapConverter converter = convertedRoom();
  EXPECT_EQ(converter.map.get(kWallX, kWallY), 100);
  EXPECT_TRUE(std::isnan(converter.map.getHeight(kWallX, kWallY)));
}

// H-05: the neighbour write pairs the neighbour's x with the centre cell's
// y. Free cells (2, 3), (3, 3) and (4, 3) reach the wall at (3, 4) along y
// or diagonally and write its occupancy to (3, 3), a free cell.
TEST(MapConverter, H05_NeighbourWriteUsesTheNeighbourY) {
  MapConverter converter = convertedRoom();
  EXPECT_EQ(converter.map.get(kWallX, kWallY - 1), 0);
  EXPECT_EQ(converter.map.get(kWallX, kWallY), 100);
}

// H-05: two walls that free cells reach along x, along y and diagonally,
// so most neighbour writes have a neighbour y that differs from the centre
// y. Every cell of the evaluated area must hold its own value: the walls
// 100, the free cells 0.
TEST(MapConverter, H05_EveryNeighbourWriteHitsOnlyTheWall) {
  const std::vector<std::pair<int, int>> walls = {{3, 4}, {4, 2}};
  MapConverter converter = convertedRoom(walls);
  for (int x = 1; x < kSize - 1; ++x) {
    for (int y = 1; y < kSize - 1; ++y) {
      const bool wall = std::find(walls.begin(), walls.end(),
                                  std::make_pair(x, y)) != walls.end();
      EXPECT_EQ(converter.map.get(x, y), wall ? 100 : 0) << x << "," << y;
    }
  }
}

}  // namespace
