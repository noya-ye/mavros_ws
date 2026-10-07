#include "ego_2d_planner_pkg/path_searching/astar_2d.hpp"

namespace ego_2d_planner_pkg
{

bool AStar2D::search(const GridMap2D& map,
                     const Vec2& start,
                     const Vec2& goal,
                     std::vector<Vec2>& path,
                     int max_iter)
{
  path.clear();

  int sx = 0, sy = 0, gx = 0, gy = 0;
  if (!map.worldToGrid(start, sx, sy) || !map.worldToGrid(goal, gx, gy)) return false;
  if (map.isOccupied(sx, sy) || map.isOccupied(gx, gy)) return false;

  const int w = map.width();
  const int h = map.height();
  const int n = w * h;

  g_score_.assign(n, std::numeric_limits<double>::infinity());
  parent_.assign(n, -1);
  closed_.assign(n, 0);
  auto& g_score = g_score_;
  auto& parent = parent_;
  auto& closed = closed_;

  auto idx = [w](int x, int y) { return y * w + x; };
  auto heuristic = [gx, gy](int x, int y) { return std::hypot(x - gx, y - gy); };

  open_.clear();
  auto& open = open_;
  const int sidx = idx(sx, sy);
  g_score[sidx] = 0.0;
  open.push_back(Node{sx, sy, heuristic(sx, sy)});
  std::push_heap(open.begin(), open.end(), Cmp{});

  const int dirs[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };

  int found_idx = -1;
  int iter = 0;

  while (!open.empty() && iter++ < max_iter) {
    const Node cur = open.front();
    std::pop_heap(open.begin(), open.end(), Cmp{});
    open.pop_back();
    const int cidx = idx(cur.x, cur.y);
    if (closed[cidx]) continue;
    closed[cidx] = 1;

    if (cur.x == gx && cur.y == gy) {
      found_idx = cidx;
      break;
    }

    for (const auto& d : dirs) {
      const int nx = cur.x + d[0];
      const int ny = cur.y + d[1];
      if (!map.inMap(nx, ny) || map.isOccupied(nx, ny)) continue;

      // 防止斜向贴角穿障碍。
      if (d[0] != 0 && d[1] != 0) {
        if (map.isOccupied(cur.x + d[0], cur.y) || map.isOccupied(cur.x, cur.y + d[1])) continue;
      }

      const int nidx = idx(nx, ny);
      if (closed[nidx]) continue;

      const double step = (d[0] == 0 || d[1] == 0) ? 1.0 : 1.41421356237;
      const double ng = g_score[cidx] + step;
      if (ng < g_score[nidx]) {
        g_score[nidx] = ng;
        parent[nidx] = cidx;
        open.push_back(Node{nx, ny, ng + heuristic(nx, ny)});
        std::push_heap(open.begin(), open.end(), Cmp{});
      }
    }
  }

  if (found_idx < 0) return false;

  std::vector<Vec2> rev;
  int cur = found_idx;
  while (cur >= 0) {
    const int x = cur % w;
    const int y = cur / w;
    rev.push_back(map.gridToWorld(x, y));
    if (cur == sidx) break;
    cur = parent[cur];
  }

  if (rev.empty()) return false;
  std::reverse(rev.begin(), rev.end());
  rev.front() = start;
  rev.back() = goal;
  path = std::move(rev);
  return true;
}

}  // namespace ego_2d_planner_pkg
