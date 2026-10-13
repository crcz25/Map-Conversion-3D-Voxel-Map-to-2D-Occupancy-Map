// Map2D: the tiled, growing 2D map behind /mapUAV, the height maps and the
// slope map. Characterization of today's behaviour, and the H-06 tests
// (growth toward negative coordinates).
//
// M-30: the tiles are std::vector cells owned by the map; these tests leave
// no leak under LeakSanitizer.

#include <gtest/gtest.h>

#include <cmath>
#include <type_traits>
#include <utility>
#include <vector>

#include "Map2D.hh"

namespace {

TEST(Map2D, NewMapIsEmptyAndReadsUnknown) {
  Map2D m(1.0);
  EXPECT_EQ(m.sizeX(), 0);
  EXPECT_EQ(m.sizeY(), 0);
  EXPECT_DOUBLE_EQ(m.offsetX(), 0.0);
  EXPECT_DOUBLE_EQ(m.offsetY(), 0.0);
  EXPECT_DOUBLE_EQ(m.getResulution(), 1.0);
  EXPECT_EQ(m.get(0, 0), -1);
  EXPECT_TRUE(std::isnan(m.getHeight(0, 0)));
  EXPECT_TRUE(m.get().empty());
}

TEST(Map2D, SetByIndexGrowsTheMapAndOtherCellsStayUnknown) {
  Map2D m(1.0);
  m.set(2, 3, 7);
  EXPECT_EQ(m.sizeX(), 3);
  EXPECT_EQ(m.sizeY(), 4);
  EXPECT_EQ(m.get(2, 3), 7);
  EXPECT_EQ(m.get(0, 0), -1);
  EXPECT_EQ(m.get(1, 3), -1);
  // Outside the map.
  EXPECT_EQ(m.get(3, 3), -1);
  EXPECT_EQ(m.get(-1, 0), -1);
  EXPECT_TRUE(std::isnan(m.getHeight(2, 3)));
  EXPECT_TRUE(std::isnan(m.getSlope(2, 3)));
}

TEST(Map2D, GetAllIsRowMajorFromTheOrigin) {
  Map2D m(1.0);
  m.set(0, 0, 1);
  m.set(1, 0, 2);
  m.set(0, 1, 3);
  m.set(1, 1, 4);
  EXPECT_EQ(m.get(), (std::vector<int>{1, 2, 3, 4}));
}

TEST(Map2D, WorldCoordinatesUseResolutionAndOffset) {
  Map2D m(0.5, -1.0, -2.0);
  m.set(0.0, 0.0, 4);  // index ((0 - -1) / 0.5, (0 - -2) / 0.5) = (2, 4)
  EXPECT_EQ(m.get(2, 4), 4);
  EXPECT_EQ(m.get(0.0, 0.0), 4);
  m.setHeight(0.0, 0.0, 1.25);
  m.setHeightTop(0.0, 0.0, 3.5);
  EXPECT_DOUBLE_EQ(m.getHeight(2, 4), 1.25);
  EXPECT_DOUBLE_EQ(m.getHeightTop(2, 4), 3.5);
  EXPECT_DOUBLE_EQ(m.offsetX(), -1.0);
  EXPECT_DOUBLE_EQ(m.offsetY(), -2.0);
}

TEST(Map2D, GrowthTowardPositiveCoordinatesKeepsEveryCell) {
  Map2D m(1.0);
  m.set(1, 0, 9);
  m.set(0, 1, 8);
  // Past the first 128 x 128 tile in x and in y.
  m.set(200, 0, 5);
  m.set(0, 300, 6);
  EXPECT_EQ(m.sizeX(), 201);
  EXPECT_EQ(m.sizeY(), 301);
  EXPECT_DOUBLE_EQ(m.offsetX(), 0.0);
  EXPECT_DOUBLE_EQ(m.offsetY(), 0.0);
  EXPECT_EQ(m.get(1, 0), 9);
  EXPECT_EQ(m.get(0, 1), 8);
  EXPECT_EQ(m.get(200, 0), 5);
  EXPECT_EQ(m.get(0, 300), 6);
  EXPECT_EQ(m.get(100, 100), -1);
}

TEST(Map2D, GrowthTowardNegativeCoordinatesMovesTheOffset) {
  Map2D m(1.0);
  m.set(0.0, 0.0, 7);
  m.set(-1.0, 0.0, 5);
  EXPECT_EQ(m.sizeX(), 2);
  EXPECT_DOUBLE_EQ(m.offsetX(), -1.0);
  EXPECT_DOUBLE_EQ(m.offsetY(), 0.0);
}

// H-06: growing toward negative coordinates added SUBMAP_SIZE + 1 to the
// tile offset instead of SUBMAP_SIZE, so every cell moved one index: world
// x = -1 then read the value written at x = 0.
TEST(Map2D, H06_GrowthTowardNegativeKeepsCellsAtTheirPosition) {
  Map2D m(1.0);
  m.set(0.0, 0.0, 7);
  m.set(-1.0, 0.0, 5);
  EXPECT_EQ(m.get(0.0, 0.0), 7);
  EXPECT_EQ(m.get(-1.0, 0.0), 5);
}

// H-06: grow toward -x, then -y, then both, each time by more than one
// tile (SUBMAP_SIZE = 128 cells). After every growth each cell written so
// far keeps its value and height at its world position, and its index
// counted from the map origin (info.origin = offsetX(), offsetY()) is
// (world - origin) / resolution.
TEST(Map2D, H06_RepeatedNegativeGrowthKeepsEveryCell) {
  struct Cell {
    double x, y;
    int value;
  };
  Map2D m(1.0);
  std::vector<Cell> cells;
  const auto write = [&](double x, double y) {
    const int value = static_cast<int>(cells.size()) % 100 + 1;
    m.set(x, y, value);
    m.setHeight(x, y, 0.5 * value);
    cells.push_back({x, y, value});
  };
  const auto check = [&](const char *step) {
    for (const Cell &c : cells) {
      EXPECT_EQ(m.get(c.x, c.y), c.value) << step << ": " << c.x << "," << c.y;
      const int ix = static_cast<int>(c.x - m.offsetX());
      const int iy = static_cast<int>(c.y - m.offsetY());
      EXPECT_EQ(m.get(ix, iy), c.value) << step << ": " << c.x << "," << c.y;
      EXPECT_DOUBLE_EQ(m.getHeight(ix, iy), 0.5 * c.value)
          << step << ": " << c.x << "," << c.y;
    }
  };
  write(0, 0);
  write(5, 3);
  write(200, 140);  // growth toward +x and +y
  check("positive");
  for (const auto &[x, y] : std::vector<std::pair<double, double>>{
           {-1, 0}, {-130, 2}, {-400, 1}}) {
    write(x, y);
    check("negative x");
  }
  EXPECT_DOUBLE_EQ(m.offsetX(), -400.0);
  for (const auto &[x, y] : std::vector<std::pair<double, double>>{
           {3, -1}, {-2, -129}, {7, -300}}) {
    write(x, y);
    check("negative y");
  }
  EXPECT_DOUBLE_EQ(m.offsetY(), -300.0);
  for (const auto &[x, y] : std::vector<std::pair<double, double>>{
           {-401, -301}, {-700, -650}, {-701, -651}}) {
    write(x, y);
    check("negative x and y");
  }
  EXPECT_DOUBLE_EQ(m.offsetX(), -701.0);
  EXPECT_DOUBLE_EQ(m.offsetY(), -651.0);
  EXPECT_EQ(m.sizeX(), 200 + 701 + 1);
  EXPECT_EQ(m.sizeY(), 140 + 651 + 1);
}

// M-30: a copy would duplicate every tile; no code needs one, so Map2D is
// move-only.
static_assert(!std::is_copy_constructible_v<Map2D>, "Map2D is not copyable");
static_assert(!std::is_copy_assignable_v<Map2D>, "Map2D is not copyable");
static_assert(std::is_move_constructible_v<Map2D>, "Map2D is movable");
static_assert(std::is_move_assignable_v<Map2D>, "Map2D is movable");

TEST(Map2D, M30_AMovedMapKeepsEveryCell) {
  Map2D m(1.0);
  m.set(-200.0, 3.0, 7);
  m.setHeight(-200.0, 3.0, 1.5);
  m.set(150.0, -140.0, 9);
  Map2D moved(std::move(m));
  EXPECT_EQ(moved.get(-200.0, 3.0), 7);
  EXPECT_EQ(moved.get(150.0, -140.0), 9);
  EXPECT_DOUBLE_EQ(moved.getHeight(0, static_cast<int>(3.0 - moved.offsetY())), 1.5);
  Map2D assigned(1.0);
  assigned = std::move(moved);
  EXPECT_EQ(assigned.get(150.0, -140.0), 9);
}

// M-30: a map grown in every direction over many tiles, then destroyed; in
// the sanitizer build LeakSanitizer reports nothing for it.
TEST(Map2D, M30_AGrownMapReleasesItsTiles) {
  {
    Map2D m(1.0);
    for (int k = 0; k < 6; ++k) {
      const double far = 130.0 * (k + 1);
      m.set(far, far, k);
      m.set(-far, -far, k);
      m.setHeight(far, -far, 0.5);
      m.setHeightTop(-far, far, 2.0);
      m.setSlope(0.0, far, 0.1);
    }
    EXPECT_EQ(m.get(780.0, 780.0), 5);
    EXPECT_EQ(m.get(-780.0, -780.0), 5);
  }
}

TEST(Map2D, UpdateSlopeFitsAPlaneThroughTheHeights) {
  Map2D m(1.0);
  // Floor rising 0.5 per cell in x, flat in y.
  for (int x = 0; x < 5; ++x) {
    for (int y = 0; y < 5; ++y) {
      m.setHeight(x, y, 0.5 * x);
    }
  }
  m.updateSlope(1, 0, 5, 0, 5);
  EXPECT_NEAR(m.getSlope(2, 2), 0.5, 1e-9);
  // The border is not updated.
  EXPECT_TRUE(std::isnan(m.getSlope(0, 2)));
  // Above the threshold the slope-filtered map reports an obstacle.
  m.set(2, 2, 0);
  EXPECT_EQ(m.get(2, 2, 0.4), 100);
  EXPECT_EQ(m.get(2, 2, 0.6), 0);
}

}  // namespace
