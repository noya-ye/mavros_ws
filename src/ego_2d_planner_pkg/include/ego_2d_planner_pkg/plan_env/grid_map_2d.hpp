#pragma once

#include <vector>
#include <array>
#include <cstdint>
#include <utility>
#include <unordered_map>
#include "ego_2d_planner_pkg/common/types.hpp"

namespace ego_2d_planner_pkg
{

class GridMap2D
{
public:
  void configure(const PlannerParams2D& p);
  void resetAround(const Vec2& center);

  bool worldToGrid(const Vec2& p, int& ix, int& iy) const;
  Vec2 gridToWorld(int ix, int iy) const;
  bool inMap(int ix, int iy) const;
  int index(int ix, int iy) const;

  // Frame update API:
  // beginUpdate() -> setOccupiedWorld() many times -> finishUpdate()
  // finishUpdate() inserts confirmed persistent cells into raw_data_ before inflation.
  void beginUpdate(double stamp_s);
  void setOccupiedWorld(const Vec2& p);
  void finishUpdate();

  void inflateObstacles();
  void computeDistanceField();

  bool isOccupied(int ix, int iy) const;
  bool isOccupiedWorld(const Vec2& p) const;
  bool isSegmentSafe(const Vec2& a, const Vec2& b, double step) const;
  bool isPathSafe(const std::vector<Vec2>& path, double step) const;

  double getDistanceGrid(int ix, int iy) const;
  double getDistanceWorld(const Vec2& p) const;
  Vec2 getDistanceGradientWorld(const Vec2& p) const;

  const std::vector<int8_t>& data() const { return data_; }
  const std::vector<int8_t>& rawData() const { return raw_data_; }
  const std::vector<double>& distData() const { return dist_data_; }
  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  double origin_x() const { return origin_x_; }
  double origin_y() const { return origin_y_; }

  std::size_t persistentCellCount() const { return persistent_cell_count_; }
  std::size_t persistentCandidateCount() const { return persistent_candidates_.size(); }

private:
  struct PersistentCandidate
  {
    double first_seen_s{0.0};
    double last_seen_s{0.0};
    int observations{0};
  };

  struct InflateRow
  {
    int dy;
    int min_dx;
    int max_dx;
  };

  static constexpr int kPersistentBlockSize = 16;
  static int blockCoordinate(int world_cell);

  int worldCellX(double x) const;
  int worldCellY(double y) const;
  std::int64_t worldKey(int wx, int wy) const;
  std::pair<int, int> decodeWorldKey(std::int64_t key) const;
  Vec2 worldCellCenter(int wx, int wy) const;
  void updatePersistentCandidate(std::int64_t key);
  void applyPersistentOccupiedToRaw();

private:
  double resolution_{0.10};
  double size_x_{12.0};
  double size_y_{12.0};
  double origin_x_{-6.0};
  double origin_y_{-6.0};
  double inflate_radius_{0.30};
  int width_{120};
  int height_{120};
  int inflate_cells_{3};
  int occupied_threshold_{50};

  bool persistent_cache_enable_{false};
  double persistent_confirm_s_{2.0};
  double persistent_miss_tolerance_s_{0.35};
  int persistent_min_observations_{3};
  double update_stamp_s_{0.0};

  std::unordered_map<std::int64_t, PersistentCandidate> persistent_candidates_;
  std::unordered_map<std::int64_t, std::array<std::uint64_t, 4>> persistent_blocks_;
  std::size_t persistent_cell_count_{0};
  std::vector<std::int64_t> observed_keys_this_frame_;

  std::vector<int8_t> data_;
  std::vector<int8_t> raw_data_;
  std::vector<double> dist_data_;
  std::vector<InflateRow> inflate_rows_;
};

}  // namespace ego_2d_planner_pkg
