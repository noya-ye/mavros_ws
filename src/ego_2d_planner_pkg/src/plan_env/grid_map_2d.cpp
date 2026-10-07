#include "ego_2d_planner_pkg/plan_env/grid_map_2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ego_2d_planner_pkg
{

void GridMap2D::configure(const PlannerParams2D& p)
{
  resolution_ = std::max(0.02, p.resolution);//栅格边长
  size_x_ = std::max(1.0, p.map_size_x);//地图宽度
  size_y_ = std::max(1.0, p.map_size_y);//地图长度
  inflate_radius_ = std::max(0.0, p.inflate_radius);//膨胀半径
  occupied_threshold_ = p.occupied_threshold;//判定阈值

  persistent_cache_enable_ = p.persistent_cache_enable;//持久障碍物判定
  persistent_confirm_s_ = std::max(0.0, p.persistent_confirm_s);
  persistent_miss_tolerance_s_ = std::max(0.0, p.persistent_miss_tolerance_s);
  persistent_min_observations_ = std::max(1, p.persistent_min_observations);

  width_ = static_cast<int>(std::ceil(size_x_ / resolution_));//栅格地图宽度
  height_ = static_cast<int>(std::ceil(size_y_ / resolution_));//栅格地图长度
  inflate_cells_ = static_cast<int>(std::ceil(inflate_radius_ / resolution_));
  data_.assign(width_ * height_, 0);
  raw_data_.assign(width_ * height_, 0);
  dist_data_.assign(width_ * height_, 1e6);

  inflate_rows_.clear();
  const int max_dx = std::min(inflate_cells_, width_ - 1);
  const int max_dy = std::min(inflate_cells_, height_ - 1);
  for (int dy = -max_dy; dy <= max_dy; ++dy) {
    int min_dx = max_dx + 1;
    int row_max_dx = -max_dx - 1;
    for (int dx = -max_dx; dx <= max_dx; ++dx) {
      const double d = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * resolution_;
      if (d <= inflate_radius_) {
        min_dx = std::min(min_dx, dx);
        row_max_dx = dx;
      }
    }
    if (min_dx <= row_max_dx) inflate_rows_.push_back({dy, min_dx, row_max_dx});
  }

  persistent_candidates_.clear();
  persistent_blocks_.clear();
  persistent_cell_count_ = 0;
  observed_keys_this_frame_.clear();
}

void GridMap2D::resetAround(const Vec2& center)
{
  origin_x_ = center.x - size_x_ * 0.5;
  origin_y_ = center.y - size_y_ * 0.5;
  std::fill(data_.begin(), data_.end(), 0);
  std::fill(raw_data_.begin(), raw_data_.end(), 0);
  std::fill(dist_data_.begin(), dist_data_.end(), 1e6);
  observed_keys_this_frame_.clear();
}

bool GridMap2D::worldToGrid(const Vec2& p, int& ix, int& iy) const
{
  ix = static_cast<int>(std::floor((p.x - origin_x_) / resolution_));
  iy = static_cast<int>(std::floor((p.y - origin_y_) / resolution_));
  return inMap(ix, iy);
}

Vec2 GridMap2D::gridToWorld(int ix, int iy) const
{
  return Vec2{origin_x_ + (static_cast<double>(ix) + 0.5) * resolution_,
              origin_y_ + (static_cast<double>(iy) + 0.5) * resolution_};
}

bool GridMap2D::inMap(int ix, int iy) const
{
  return ix >= 0 && iy >= 0 && ix < width_ && iy < height_;
}

int GridMap2D::index(int ix, int iy) const
{
  return iy * width_ + ix;
}

int GridMap2D::worldCellX(double x) const
{
  return static_cast<int>(std::floor(x / resolution_));
}

int GridMap2D::worldCellY(double y) const
{
  return static_cast<int>(std::floor(y / resolution_));
}

std::int64_t GridMap2D::worldKey(int wx, int wy) const
{
  const std::uint64_t ux = static_cast<std::uint32_t>(wx);
  const std::uint64_t uy = static_cast<std::uint32_t>(wy);
  return static_cast<std::int64_t>((ux << 32) | uy);
}

std::pair<int, int> GridMap2D::decodeWorldKey(std::int64_t key) const
{
  const std::uint64_t ukey = static_cast<std::uint64_t>(key);
  const int wx = static_cast<int>(static_cast<std::int32_t>(static_cast<std::uint32_t>(ukey >> 32)));
  const int wy = static_cast<int>(static_cast<std::int32_t>(static_cast<std::uint32_t>(ukey & 0xffffffffULL)));
  return {wx, wy};
}

Vec2 GridMap2D::worldCellCenter(int wx, int wy) const
{
  return Vec2{(static_cast<double>(wx) + 0.5) * resolution_,
              (static_cast<double>(wy) + 0.5) * resolution_};
}

int GridMap2D::blockCoordinate(int world_cell)
{
  // Integer division truncates toward zero; negative world cells need floor.
  const int quotient = world_cell / kPersistentBlockSize;
  return quotient - (world_cell % kPersistentBlockSize < 0 ? 1 : 0);
}

void GridMap2D::beginUpdate(double stamp_s)
{
  update_stamp_s_ = stamp_s;
  observed_keys_this_frame_.clear();
}

void GridMap2D::setOccupiedWorld(const Vec2& p)
{
  int ix = 0, iy = 0;
  if (!worldToGrid(p, ix, iy)) return;

  raw_data_[index(ix, iy)] = 100;

  if (persistent_cache_enable_) {
    const int wx = worldCellX(p.x);
    const int wy = worldCellY(p.y);
    observed_keys_this_frame_.push_back(worldKey(wx, wy));
  }
}

void GridMap2D::updatePersistentCandidate(std::int64_t key)
{
  const auto [wx, wy] = decodeWorldKey(key);
  const int bx = blockCoordinate(wx);
  const int by = blockCoordinate(wy);
  const auto block_key = worldKey(bx, by);
  const auto local_key = static_cast<std::uint8_t>(
    (wy - by * kPersistentBlockSize) * kPersistentBlockSize + wx - bx * kPersistentBlockSize);
  const auto bit = std::uint64_t{1} << (local_key % 64);
  const auto block_it = persistent_blocks_.find(block_key);
  if (block_it != persistent_blocks_.end()) {
    if (block_it->second[local_key / 64] & bit) return;
  }

  auto it = persistent_candidates_.find(key);
  if (it == persistent_candidates_.end() ||
      update_stamp_s_ - it->second.last_seen_s > persistent_miss_tolerance_s_) {
    PersistentCandidate cand;
    cand.first_seen_s = update_stamp_s_;
    cand.last_seen_s = update_stamp_s_;
    cand.observations = 1;
    persistent_candidates_[key] = cand;
    return;
  }

  it->second.last_seen_s = update_stamp_s_;
  it->second.observations += 1;

  const double continuous_time = it->second.last_seen_s - it->second.first_seen_s;
  if (continuous_time >= persistent_confirm_s_ &&
      it->second.observations >= persistent_min_observations_) {
    persistent_blocks_[block_key][local_key / 64] |= bit;
    ++persistent_cell_count_;
    persistent_candidates_.erase(it);
  }
}

void GridMap2D::applyPersistentOccupiedToRaw()
{
  if (!persistent_cache_enable_) {
    return;
  }

  // Include a cell beyond each bound to cover grid alignment and rounding;
  // worldToGrid remains the authoritative test, exactly as for the full cache.
  const int min_bx = blockCoordinate(worldCellX(origin_x_) - 1);
  const int max_bx = blockCoordinate(worldCellX(origin_x_ + width_ * resolution_) + 1);
  const int min_by = blockCoordinate(worldCellY(origin_y_) - 1);
  const int max_by = blockCoordinate(worldCellY(origin_y_ + height_ * resolution_) + 1);
  for (int by = min_by; by <= max_by; ++by) {
    for (int bx = min_bx; bx <= max_bx; ++bx) {
      const auto it = persistent_blocks_.find(worldKey(bx, by));
      if (it == persistent_blocks_.end()) continue;
      for (int word = 0; word < 4; ++word) {
        auto occupied = it->second[word];
        while (occupied != 0) {
          const int local_key = word * 64 + __builtin_ctzll(occupied);
          const int wx = bx * kPersistentBlockSize + local_key % kPersistentBlockSize;
          const int wy = by * kPersistentBlockSize + local_key / kPersistentBlockSize;
          const Vec2 p = worldCellCenter(wx, wy);
          int ix = 0, iy = 0;
          if (worldToGrid(p, ix, iy)) raw_data_[index(ix, iy)] = 100;
          occupied &= occupied - 1;
        }
      }
    }
  }
}

void GridMap2D::finishUpdate()
{
  if (!persistent_cache_enable_) {
    return;
  }

  std::sort(observed_keys_this_frame_.begin(), observed_keys_this_frame_.end());
  const auto unique_end = std::unique(observed_keys_this_frame_.begin(), observed_keys_this_frame_.end());
  observed_keys_this_frame_.erase(unique_end, observed_keys_this_frame_.end());
  for (const auto key : observed_keys_this_frame_) {
    updatePersistentCandidate(key);
  }

  // Remove stale unconfirmed candidates. Confirmed occupied cells are permanent
  // during this node's lifetime and are intentionally not removed here.
  for (auto it = persistent_candidates_.begin(); it != persistent_candidates_.end();) {
    if (update_stamp_s_ - it->second.last_seen_s > persistent_miss_tolerance_s_) {
      it = persistent_candidates_.erase(it);
    } else {
      ++it;
    }
  }

  applyPersistentOccupiedToRaw();
}

void GridMap2D::inflateObstacles()
{
  data_ = raw_data_;
  if (inflate_cells_ <= 0) return;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (raw_data_[index(x, y)] < occupied_threshold_) continue;
      for (const auto& row : inflate_rows_) {
        const int ny = y + row.dy;
        if (ny < 0 || ny >= height_) continue;
        const int min_x = std::max(0, x + row.min_dx);
        const int max_x = std::min(width_ - 1, x + row.max_dx);
        std::fill(data_.begin() + index(min_x, ny), data_.begin() + index(max_x, ny) + 1, 100);
      }
    }
  }
}

void GridMap2D::computeDistanceField()
{
  for (std::size_t i = 0; i < data_.size(); ++i) {
    dist_data_[i] = data_[i] >= occupied_threshold_ ? 0.0 : 1e6;
  }

  // With no propagation barriers, two sweeps give the same eight-neighbor
  // distance metric as Dijkstra, retaining its original diagonal weight.
  const int forward[4][2] = {{-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
  const int backward[4][2] = {{1, 0}, {1, 1}, {0, 1}, {-1, 1}};
  const double diagonal_step = resolution_ * 1.41421356237;
  auto relax = [&](int x, int y, const int (&dirs)[4][2]) {
    double& best = dist_data_[index(x, y)];
    if (best == 0.0) return;
    for (const auto& dir : dirs) {
      const int nx = x + dir[0];
      const int ny = y + dir[1];
      if (!inMap(nx, ny)) continue;
      const double step = (dir[0] == 0 || dir[1] == 0) ? resolution_ : diagonal_step;
      const double nd = dist_data_[index(nx, ny)] + step;
      if (nd < best) best = nd;
    }
  };

  for (int y = 0; y < height_; ++y) {
    if (y == 0 || width_ < 3) {
      for (int x = 0; x < width_; ++x) relax(x, y, forward);
      continue;
    }
    relax(0, y, forward);
    double* row = dist_data_.data() + y * width_;
    const double* previous = row - width_;
    for (int x = 1; x < width_ - 1; ++x) {
      double& best = row[x];
      if (best == 0.0) continue;
      double candidate = row[x - 1] + resolution_;
      if (candidate < best) best = candidate;
      candidate = previous[x - 1] + diagonal_step;
      if (candidate < best) best = candidate;
      candidate = previous[x] + resolution_;
      if (candidate < best) best = candidate;
      candidate = previous[x + 1] + diagonal_step;
      if (candidate < best) best = candidate;
    }
    relax(width_ - 1, y, forward);
  }
  for (int y = height_ - 1; y >= 0; --y) {
    if (y == height_ - 1 || width_ < 3) {
      for (int x = width_ - 1; x >= 0; --x) relax(x, y, backward);
      continue;
    }
    relax(width_ - 1, y, backward);
    double* row = dist_data_.data() + y * width_;
    const double* next = row + width_;
    for (int x = width_ - 2; x > 0; --x) {
      double& best = row[x];
      if (best == 0.0) continue;
      double candidate = row[x + 1] + resolution_;
      if (candidate < best) best = candidate;
      candidate = next[x + 1] + diagonal_step;
      if (candidate < best) best = candidate;
      candidate = next[x] + resolution_;
      if (candidate < best) best = candidate;
      candidate = next[x - 1] + diagonal_step;
      if (candidate < best) best = candidate;
    }
    relax(0, y, backward);
  }
}

bool GridMap2D::isOccupied(int ix, int iy) const
{
  if (!inMap(ix, iy)) return true;
  return data_[index(ix, iy)] >= occupied_threshold_;
}

bool GridMap2D::isOccupiedWorld(const Vec2& p) const
{
  int ix = 0, iy = 0;
  if (!worldToGrid(p, ix, iy)) return true;
  return isOccupied(ix, iy);
}

bool GridMap2D::isSegmentSafe(const Vec2& a, const Vec2& b, double step) const
{
  const double len = dist(a, b);
  const int n = std::max(1, static_cast<int>(std::ceil(len / std::max(0.02, step))));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    const Vec2 p = a + (b - a) * t;
    if (isOccupiedWorld(p)) return false;
  }
  return true;
}

bool GridMap2D::isPathSafe(const std::vector<Vec2>& path, double step) const
{
  if (path.empty()) return false;
  if (path.size() == 1) return !isOccupiedWorld(path.front());
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (!isSegmentSafe(path[i - 1], path[i], step)) return false;
  }
  return true;
}

double GridMap2D::getDistanceGrid(int ix, int iy) const
{
  if (!inMap(ix, iy)) return 0.0;
  return dist_data_[index(ix, iy)];
}

double GridMap2D::getDistanceWorld(const Vec2& p) const
{
  int ix = 0, iy = 0;
  if (!worldToGrid(p, ix, iy)) return 0.0;
  return getDistanceGrid(ix, iy);
}

Vec2 GridMap2D::getDistanceGradientWorld(const Vec2& p) const
{
  int ix = 0, iy = 0;
  if (!worldToGrid(p, ix, iy)) return {0.0, 0.0};

  const double dx = (getDistanceGrid(ix + 1, iy) - getDistanceGrid(ix - 1, iy)) / (2.0 * resolution_);
  const double dy = (getDistanceGrid(ix, iy + 1) - getDistanceGrid(ix, iy - 1)) / (2.0 * resolution_);
  Vec2 g{dx, dy};
  const double n = norm(g);
  if (n < 1e-6 || !std::isfinite(n)) return {0.0, 0.0};
  return g / n;
}

}  // namespace ego_2d_planner_pkg
