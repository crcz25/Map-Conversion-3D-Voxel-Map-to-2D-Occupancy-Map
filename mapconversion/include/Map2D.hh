#pragma once
#include "datatypes.hh"

#include <utility>
const int SUBMAP_SIZE = 128;

class Map2D {
public:
  Map2D(double resolution = 0, double offsetX = 0, double offsetY = 0) {
    Map2D::resolution = resolution;
    setOffset(offsetX, offsetY);
    mapSizeX = 0;
    mapSizeY = 0;
    subMapOffsetX = 0;
    subMapOffsetY = 0;
    growSubMaps(1, 1);

    MatrixXd A(9, 3);
    vector<point3D> points;
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        point3D p = {double(dx), double(dy), 0};
        points.push_back(p);
      }
    }
    for (int i = 0; i < 9; ++i) {
      A(i, 0) = points[i].x;
      A(i, 1) = points[i].y;
      A(i, 2) = 1;
    }

    MatrixXd AtA = A.transpose() * A;
    MatrixXd AtA_inv = AtA.inverse();
    MatrixXd At = A.transpose();
    preCalcA = AtA_inv * At;
  }

  // M-30: no code copies a Map2D (a copy would duplicate every tile); it is
  // moved instead.
  Map2D(const Map2D &) = delete;
  Map2D &operator=(const Map2D &) = delete;
  Map2D(Map2D &&) = default;
  Map2D &operator=(Map2D &&) = default;

  // get all occupancy data
  vector<int> get() {
    vector<int> newData;
    for (int y = 0; y < sizeY(); y++) {
      for (int x = 0; x < sizeX(); x++) {
        newData.push_back(get(x, y));
      }
    }
    return newData;
  }

  // get all occupancy data filtered through slope threshold
  vector<int> get(double slope) {
    vector<int> newData;
    for (int y = 0; y < sizeY(); y++) {
      for (int x = 0; x < sizeX(); x++) {
        double newSlope = getSlope(x, y);
        if (!isnan(newSlope) && newSlope > slope) {
          newData.push_back(100);
        } else {
          newData.push_back(get(x, y));
        }
      }
    }
    return newData;
  }

  // get occupancy data at the x y position
  int get(double x, double y) {
    int posX = (x - mapOffsetX) / resolution;
    int posY = (y - mapOffsetY) / resolution;
    return get(posX, posY);
  }
  // get occupancy data at the x y index
  int get(int x, int y) {
    if (x < 0 || sizeX() <= x)
      return -1;
    if (y < 0 || sizeY() <= y)
      return -1;
    x += subMapOffsetX;
    y += subMapOffsetY;
    int subIndexX = x / SUBMAP_SIZE;
    int subIndexY = y / SUBMAP_SIZE;
    if (subMap[subIndexX][subIndexY].empty())
      return -1;
    int indexX = x % SUBMAP_SIZE;
    int indexY = y % SUBMAP_SIZE;
    return subMap[subIndexX][subIndexY][cell(indexX, indexY)];
  }
  // get occupancy data at x y filtered through slope threshold
  int get(int x, int y, double maxSlope) {
    if (getSlope(x, y) > maxSlope)
      return 100;
    return get(x, y);
  }

  // get all floor data
  vector<double> getHeight() {
    vector<double> newData;
    for (int y = 0; y < sizeY(); y++) {
      for (int x = 0; x < sizeX(); x++) {
        newData.push_back(getHeight(x, y));
      }
    }
    return newData;
  }
  // get floor data at x y
  double getHeight(int x, int y) {
    if (x < 0 || sizeX() <= x)
      return NAN;
    if (y < 0 || sizeY() <= y)
      return NAN;
    x += subMapOffsetX;
    y += subMapOffsetY;
    int subIndexX = x / SUBMAP_SIZE;
    int subIndexY = y / SUBMAP_SIZE;
    if (subMapHeight[subIndexX][subIndexY].empty())
      return NAN;
    int indexX = x % SUBMAP_SIZE;
    int indexY = y % SUBMAP_SIZE;
    return subMapHeight[subIndexX][subIndexY][cell(indexX, indexY)];
  }

  // get all ceiling data
  vector<double> getHeightTop() {
    vector<double> newData;
    for (int y = 0; y < sizeY(); y++) {
      for (int x = 0; x < sizeX(); x++) {
        newData.push_back(getHeightTop(x, y));
      }
    }
    return newData;
  }
  // get ceiling data at x y
  double getHeightTop(int x, int y) {
    if (x < 0 || sizeX() <= x)
      return NAN;
    if (y < 0 || sizeY() <= y)
      return NAN;
    x += subMapOffsetX;
    y += subMapOffsetY;
    int subIndexX = x / SUBMAP_SIZE;
    int subIndexY = y / SUBMAP_SIZE;
    if (subMapHeightTop[subIndexX][subIndexY].empty())
      return NAN;
    int indexX = x % SUBMAP_SIZE;
    int indexY = y % SUBMAP_SIZE;
    return subMapHeightTop[subIndexX][subIndexY][cell(indexX, indexY)];
  }

  // get all slope data
  vector<double> getSlope() {
    vector<double> newData;
    for (int y = 0; y < sizeY(); y++) {
      for (int x = 0; x < sizeX(); x++) {
        newData.push_back(getSlope(x, y));
      }
    }
    return newData;
  }
  // get slope data at x y
  double getSlope(int x, int y) {
    if (x < 0 || sizeX() <= x)
      return NAN;
    if (y < 0 || sizeY() <= y)
      return NAN;
    x += subMapOffsetX;
    y += subMapOffsetY;
    int subIndexX = x / SUBMAP_SIZE;
    int subIndexY = y / SUBMAP_SIZE;
    if (subMapSlope[subIndexX][subIndexY].empty())
      return NAN;
    int indexX = x % SUBMAP_SIZE;
    int indexY = y % SUBMAP_SIZE;
    return subMapSlope[subIndexX][subIndexY][cell(indexX, indexY)];
  }

  // get math width
  int sizeX() { return mapSizeX; }

  // get map hight
  int sizeY() { return mapSizeY; }

  // get x offset from frame origin
  double offsetX() { return mapOffsetX; }
  // get y offset from frame origin
  double offsetY() { return mapOffsetY; }

  double getResulution() { return resolution; }

  // set occupancy value at the x y position
  void set(double x, double y, int value) {
    int posX = (x - mapOffsetX) / resolution;
    int posY = (y - mapOffsetY) / resolution;
    set(posX, posY, value);
  }
  // set occupancy value at the x y index
  void set(int x, int y, int value) {
    int subIndexX, subIndexY, indexX, indexY;
    getIndexSubMap(x, y, subIndexX, subIndexY, indexX, indexY);
    if (subMap[subIndexX][subIndexY].empty())
      subMap[subIndexX][subIndexY] = generateSubMapInt();
    subMap[subIndexX][subIndexY][cell(indexX, indexY)] = value;
  }

  // set floor heigt value at the x y position
  void setHeight(double x, double y, double value) {
    int posX = (x - mapOffsetX) / resolution;
    int posY = (y - mapOffsetY) / resolution;
    setHeight(posX, posY, value);
  }
  // set floor heigt value at the x y index
  void setHeight(int x, int y, double value) {
    int subIndexX, subIndexY, indexX, indexY;
    getIndexSubMap(x, y, subIndexX, subIndexY, indexX, indexY);
    if (subMapHeight[subIndexX][subIndexY].empty())
      subMapHeight[subIndexX][subIndexY] = generateSubMapDouble();
    subMapHeight[subIndexX][subIndexY][cell(indexX, indexY)] = value;
  }

  // set ceiling heigt value at the x y position
  void setHeightTop(double x, double y, double value) {
    int posX = (x - mapOffsetX) / resolution;
    int posY = (y - mapOffsetY) / resolution;
    setHeightTop(posX, posY, value);
  }
  // set ceiling heigt value at the x y index
  void setHeightTop(int x, int y, double value) {
    int subIndexX, subIndexY, indexX, indexY;
    getIndexSubMap(x, y, subIndexX, subIndexY, indexX, indexY);
    if (subMapHeightTop[subIndexX][subIndexY].empty())
      subMapHeightTop[subIndexX][subIndexY] = generateSubMapDouble();
    subMapHeightTop[subIndexX][subIndexY][cell(indexX, indexY)] = value;
  }

  // set slope value at the x y position
  void setSlope(double x, double y, double value) {
    int posX = (x - mapOffsetX) / resolution;
    int posY = (y - mapOffsetY) / resolution;
    setSlope(posX, posY, value);
  }
  // set slope value at the x y index
  void setSlope(int x, int y, double value) {
    int subIndexX, subIndexY, indexX, indexY;
    getIndexSubMap(x, y, subIndexX, subIndexY, indexX, indexY);
    if (subMapSlope[subIndexX][subIndexY].empty())
      subMapSlope[subIndexX][subIndexY] = generateSubMapDouble();
    subMapSlope[subIndexX][subIndexY][cell(indexX, indexY)] = value;
  }

  void setOffset(double offsetX, double offsetY) {
    mapOffsetX = offsetX;
    mapOffsetY = offsetY;
  }

  // update slope in region of map in specified area
  void updateSlope(int areaSize, double minX, double maxX, double minY,
                   double maxY) {
    int posMinX = (minX - mapOffsetX) / resolution;
    int posMaxX = (maxX - mapOffsetX) / resolution;
    int posMinY = (minY - mapOffsetY) / resolution;
    int posMaxY = (maxY - mapOffsetY) / resolution;
    updateSlope(areaSize, posMinX, posMaxX, posMinY, posMaxY);
  }
  void updateSlope(int areaSize, int minX, int maxX, int minY, int maxY) {
    for (int x = minX + 1; x < maxX - 1; x++) {
      for (int y = minY + 1; y < maxY - 1; y++) {
        if (isnan(getHeight(x, y)))
          continue; // do not calculate slope for empty cells
        vector<point3D> points;
        for (int dx = -areaSize; dx <= areaSize; dx++) {
          for (int dy = -areaSize; dy <= areaSize; dy++) {
            if (isnan(getHeight(x + dx, y + dy)))
              continue;
            point3D p = {double(dx), double(dy), getHeight(x + dx, y + dy)};
            points.push_back(p);
          }
        }
        if (points.size() < 3)
          continue; // to slove MS problem at least 3 points is needed
        point2D p = getSlopeOfPoints(points);
        setSlope(x, y, sqrt(p.x * p.x + p.y * p.y));
      }
    }
  }

private:
  // M-30: a tile is SUBMAP_SIZE x SUBMAP_SIZE cells, row x then y
  // (cell(x, y)); an empty tile is not allocated yet (reads as unknown).
  using IntTile = vector<int>;
  using DoubleTile = vector<double>;
  static int cell(int indexX, int indexY) {
    return indexX * SUBMAP_SIZE + indexY;
  }

  vector<vector<IntTile>> subMap;
  vector<vector<DoubleTile>> subMapHeight;
  vector<vector<DoubleTile>> subMapHeightTop;
  vector<vector<DoubleTile>> subMapSlope;
  int subMapOffsetX;
  int subMapOffsetY;
  int mapSizeX, mapSizeY;
  double mapOffsetX, mapOffsetY;
  double resolution;
  MatrixXd preCalcA;

  // fits to palne and calculates slope of plane and return maximum slope
  point2D getSlopeOfPoints(vector<point3D> points) {
    int n = points.size();
    double a, b, c;
    // Construct matrix A and vector B
    MatrixXd A(n, 3);
    VectorXd B(n);
    VectorXd x(3);
    for (int i = 0; i < n; ++i) {
      B(i) = points[i].z;
    }

    if (true || n != 9) {
      for (int i = 0; i < n; ++i) {
        A(i, 0) = points[i].x;
        A(i, 1) = points[i].y;
        A(i, 2) = 1;
      }
      x = (A.transpose() * A).inverse() * A.transpose() * B;
    } else {
      x = preCalcA * B;
    }
    // Plane equation coefficients
    a = x(0);
    b = x(1);
    c = x(2);
    return {a, b};
  }

  // Get the index for map and submap, if index x y is outside of map, map is
  // expanded
  void getIndexSubMap(int x, int y, int &subIndexX, int &subIndexY, int &indexX,
                      int &indexY) {
    if (sizeX() > x && x >= 0 && sizeY() > y && y >= 0) {
      x += subMapOffsetX;
      y += subMapOffsetY;
      subIndexX = x / SUBMAP_SIZE;
      subIndexY = y / SUBMAP_SIZE;

      indexX = x % SUBMAP_SIZE;
      indexY = y % SUBMAP_SIZE;
      return;
    }
    if (sizeX() <= x) {
      mapSizeX = x + 1;
    } else if (x < 0) {
      mapSizeX -= x;
      mapOffsetX += x * resolution;
      subMapOffsetX += x;
      x = 0;
    }
    if (sizeY() <= y) {
      mapSizeY = y + 1;
    } else if (y < 0) {
      mapSizeY -= y;
      mapOffsetY += y * resolution;
      subMapOffsetY += y;
      y = 0;
    }
    x += subMapOffsetX;
    y += subMapOffsetY;
    subIndexX = x / SUBMAP_SIZE - signbit(x);
    subIndexY = y / SUBMAP_SIZE - signbit(y);

    int growX = 0, growY = 0;
    if (subMap.size() <= subIndexX) {
      growX = subIndexX - subMap.size() + 1;
    } else if (subIndexX < 0) {
      growX = subIndexX;
    }
    if (subMap[0].size() <= subIndexY) {
      growY = subIndexY - subMap[0].size() + 1;
    } else if (subIndexY < 0) {
      growY = subIndexY;
    }
    if (growX != 0 || growY != 0) {
      // Growth toward negative coordinates moves the tiles, and the tile
      // offset by the same number of cells: shift x and y with it (H-06).
      const int oldOffsetX = subMapOffsetX;
      const int oldOffsetY = subMapOffsetY;
      growSubMaps(growX, growY);
      x += subMapOffsetX - oldOffsetX;
      y += subMapOffsetY - oldOffsetY;
      subIndexX = x / SUBMAP_SIZE;
      subIndexY = y / SUBMAP_SIZE;
    }
    indexX = x % SUBMAP_SIZE;
    indexY = y % SUBMAP_SIZE;
  }

  void growSubMaps(int growX, int growY) {
    // get size of new subMaps
    int newSizeX, newSizeY;
    if (subMap.size() == 0) {
      newSizeX = abs(growX);
      newSizeY = abs(growY);
    } else {
      if (growX != 0)
        growX = growX / abs(growX) * max(int(subMap.size()), abs(growX));
      if (growY != 0)
        growY = growY / abs(growY) * max(int(subMap[0].size()), abs(growY));
      newSizeX = subMap.size() + abs(growX);
      newSizeY = subMap[0].size() + abs(growY);
    }
    // generate new subMaps
    vector<vector<IntTile>> newSubMap(newSizeX);
    vector<vector<DoubleTile>> newSubMapHeight(newSizeX);
    vector<vector<DoubleTile>> newSubMapHeightTop(newSizeX);
    vector<vector<DoubleTile>> newSubMapSlope(newSizeX);
    for (int i = 0; i < newSizeX; i++) {
      newSubMap[i] = vector<IntTile>(newSizeY);
      newSubMapHeight[i] = vector<DoubleTile>(newSizeY);
      newSubMapHeightTop[i] = vector<DoubleTile>(newSizeY);
      newSubMapSlope[i] = vector<DoubleTile>(newSizeY);
    }
    int offsetIndexX = growX < 0 ? -growX : 0;
    int offsetIndexY = growY < 0 ? -growY : 0;
    for (int x = 0; x < subMap.size(); x++) {
      for (int y = 0; y < subMap[0].size(); y++) {
        // Moves the tiles; the cells are not copied.
        newSubMap[x + offsetIndexX][y + offsetIndexY] = std::move(subMap[x][y]);
        newSubMapHeight[x + offsetIndexX][y + offsetIndexY] =
            std::move(subMapHeight[x][y]);
        newSubMapHeightTop[x + offsetIndexX][y + offsetIndexY] =
            std::move(subMapHeightTop[x][y]);
        newSubMapSlope[x + offsetIndexX][y + offsetIndexY] =
            std::move(subMapSlope[x][y]);
      }
    }
    subMapOffsetX += offsetIndexX * SUBMAP_SIZE;
    subMapOffsetY += offsetIndexY * SUBMAP_SIZE;
    subMap = std::move(newSubMap);
    subMapHeight = std::move(newSubMapHeight);
    subMapHeightTop = std::move(newSubMapHeightTop);
    subMapSlope = std::move(newSubMapSlope);
  }

  static IntTile generateSubMapInt() {
    return IntTile(std::size_t{SUBMAP_SIZE} * SUBMAP_SIZE, -1);
  }

  static DoubleTile generateSubMapDouble() {
    return DoubleTile(std::size_t{SUBMAP_SIZE} * SUBMAP_SIZE, NAN);
  }
};
