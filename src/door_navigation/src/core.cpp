#include "door_navigation/core.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <queue>
#include <stdexcept>
#include <utility>

namespace door_navigation {
void Config::validate() const {
  for (double v : {resolution, radius, corridor_width, corridor_tolerance, pre_distance,
                   post_distance, max_speed, max_acceleration, sample_dt})
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Invalid metric or motion limits");
  for (double v : {margin, wall_thickness})
    if (!std::isfinite(v) || v < 0) throw std::invalid_argument("Invalid margin or wall thickness");
  for (double v : {x_min, x_max, y_min, y_max, door_min, door_max})
    if (!std::isfinite(v)) throw std::invalid_argument("Nonfinite map or door limits");
  if (!(x_min < x_max && y_min < y_max && 0 < door_min && door_min < door_max))
    throw std::invalid_argument("Invalid map or door limits");
  if (std::min(pre_distance, post_distance) <= radius + margin + wall_thickness)
    throw std::invalid_argument("Gate offsets must clear the vehicle and wall");
}
Grid::Grid(const Config &config) : cfg(config) {
  cfg.validate();
  occupied = cv::Mat1b::zeros(static_cast<int>(std::ceil((cfg.y_max-cfg.y_min)/cfg.resolution)),
                            static_cast<int>(std::ceil((cfg.x_max-cfg.x_min)/cfg.resolution)));
  observed = cv::Mat1b::zeros(occupied.size());
}
cv::Point Grid::cell(const Vec2 &p) const {
  return {static_cast<int>(std::floor((p.x()-cfg.x_min)/cfg.resolution)),
          static_cast<int>(std::floor((p.y()-cfg.y_min)/cfg.resolution))};
}
Vec2 Grid::point(double x, double y) const {
  return {cfg.x_min+(x+0.5)*cfg.resolution, cfg.y_min+(y+0.5)*cfg.resolution};
}
bool Grid::inside(cv::Point p) const {
  return p.x >= 0 && p.x < occupied.cols && p.y >= 0 && p.y < occupied.rows;
}
void Grid::add(const Vec2 &p, const Vec2 &origin) {
  cv::Point a = cell(origin), b = cell(p), endpoint = b;
  if (cv::clipLine(occupied.size(), a, b)) cv::line(observed, a, b, cv::Scalar(1), 1);
  if (inside(endpoint)) occupied(endpoint) = 1;
}
void Grid::occupy(const Vec2 &p) {
  const auto q = cell(p);
  if (inside(q)) occupied(q) = 1;
}
Layers Grid::inflation_layers() const {
  const double radius = cfg.radius+cfg.margin;
  const int k = static_cast<int>(std::ceil(radius/cfg.resolution));
  cv::Mat1b kernel = cv::Mat1b::zeros(2*k+1, 2*k+1);
  for (int y = -k; y <= k; ++y)
    for (int x = -k; x <= k; ++x)
      kernel(y+k,x+k) = (x*x+y*y)*cfg.resolution*cfg.resolution <= radius*radius+1e-12;
  Layers result;
  cv::dilate(occupied, result.obstacles, kernel, {-1,-1}, 1, cv::BORDER_CONSTANT, 0);
  cv::Mat1b unknown;
  cv::compare(observed, 0, unknown, cv::CMP_EQ);
  unknown /= 255;
  cv::dilate(unknown, result.unknown, kernel, {-1,-1}, 1, cv::BORDER_CONSTANT, 1);
  cv::bitwise_or(result.obstacles, result.unknown, result.blocked);
  return result;
}
static double median(std::vector<double> a) {
  std::sort(a.begin(), a.end());
  const size_t n = a.size();
  return n % 2 ? a[n/2] : (a[n/2-1]+a[n/2])/2;
}
std::vector<Gate> detect_gates(const Grid &g) {
  const auto &c = g.cfg;
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(g.occupied*255, lines, 1, pi/180, 12, 0.25/c.resolution, 0.06/c.resolution);
  std::vector<double> sides, fronts;
  for (auto l : lines) {
    const int dx = l[2]-l[0], dy = l[3]-l[1];
    if (std::abs(dy) <= 0.15*std::abs(dx) && std::abs(dx)*c.resolution > 0.5)
      sides.push_back((l[1]+l[3])/2.0);
    if (std::abs(dx) <= 0.15*std::abs(dy)) fronts.push_back((l[0]+l[2])/2.0);
  }
  double error = c.corridor_tolerance;
  int lo = -1, hi = -1;
  const int cy = g.cell(Vec2::Zero()).y;
  for (double a : sides) for (double b : sides) {
    const double e = std::abs((b-a)*c.resolution-c.corridor_width);
    if (a < cy && cy < b && e < error) {
      lo = static_cast<int>(std::nearbyint(a)); hi = static_cast<int>(std::nearbyint(b)); error = e;
    }
  }
  if (lo < 0) return {};
  std::sort(fronts.begin(), fronts.end());
  std::vector<std::vector<double>> bands;
  for (double x : fronts) {
    if (g.point(x,0).x() < 0.25) continue;
    if (bands.empty() || x-bands.back().back() > 0.18/c.resolution) bands.push_back({});
    bands.back().push_back(x);
  }
  std::vector<Gate> gates;
  for (const auto &band : bands) {
    const int x = static_cast<int>(std::nearbyint(median(band)));
    const int half = std::max(1, static_cast<int>(std::ceil(c.wall_thickness/c.resolution)));
    cv::Mat1b profile = cv::Mat1b::zeros(hi-lo+1, 1);
    for (int y = lo; y <= hi; ++y)
      for (int xx = std::max(0,x-half); xx < std::min(g.occupied.cols,x+half+1); ++xx)
        profile(y-lo,0) |= g.occupied(y,xx);
    cv::morphologyEx(profile, profile, cv::MORPH_CLOSE, cv::Mat1b::ones(3,1));
    profile(0,0) = profile(profile.rows-1,0) = 1;
    for (int start = 0; start < profile.rows; ++start) {
      if (profile(start,0)) continue;
      int end = start;
      while (end < profile.rows && !profile(end,0)) ++end;
      const double width = (end-start)*c.resolution;
      if (width >= c.door_min && width <= c.door_max && width > 2*(c.radius+c.margin+c.resolution)) {
        const Vec2 a = g.point(x,lo+start-0.5), b = g.point(x,lo+end-0.5);
        gates.push_back({(a+b)/2, Vec2(1,0), width, {a,b}});
      }
      start = end;
    }
  }
  std::sort(gates.begin(), gates.end(), [](const Gate &a, const Gate &b) { return a.center.x() < b.center.x(); });
  return gates;
}
double corridor_heading(const Route &points) {
  Grid g(Config{});
  for (const auto &p : points) g.occupy(p);
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(g.occupied*255, lines, 1, pi/360, 25, 0.7/g.cfg.resolution, 0.08/g.cfg.resolution);
  std::vector<double> angles;
  for (auto l : lines) {
    double a = std::atan2(l[3]-l[1], l[2]-l[0]);
    while (a >= pi/2) a -= pi;
    while (a < -pi/2) a += pi;
    if (std::abs(a) < 20*pi/180) angles.push_back(a);
  }
  return angles.empty() ? 0 : median(angles);
}
bool clear_segment(const Grid &g, const cv::Mat1b &blocked, const Vec2 &a, const Vec2 &b) {
  const int count = std::max(2, static_cast<int>(std::ceil((b-a).norm()/(g.cfg.resolution/2)))+1);
  for (int i = 0; i < count; ++i) {
    const auto q = g.cell(a+(b-a)*(static_cast<double>(i)/(count-1)));
    if (!g.inside(q) || blocked(q)) return false;
  }
  return true;
}
Route astar(const Grid &g, const cv::Mat1b &blocked, const Vec2 &start, const Vec2 &goal) {
  const auto a = g.cell(start), b = g.cell(goal);
  if (!g.inside(a) || !g.inside(b) || blocked(a) || blocked(b)) return {};
  const int cols = blocked.cols, n = cols*blocked.rows;
  auto index = [cols](cv::Point q) { return q.y*cols+q.x; };
  auto cell = [cols](int i) { return cv::Point(i%cols,i/cols); };
  std::vector<double> costs(n, std::numeric_limits<double>::infinity());
  std::vector<int> parents(n,-1);
  std::vector<bool> closed(n,false);
  using Entry = std::pair<double,int>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
  costs[index(a)] = 0; queue.emplace(0,index(a));
  while (!queue.empty()) {
    const int ui = queue.top().second; queue.pop();
    if (closed[ui]) continue;
    closed[ui] = true;
    const auto u = cell(ui);
    if (u == b) {
      Route raw;
      for (int i = ui; i != -1; i = parents[i]) {
        const auto q = cell(i); raw.push_back(g.point(q.x,q.y));
      }
      std::reverse(raw.begin(),raw.end()); raw.insert(raw.begin(),start); raw.push_back(goal);
      Route result{start};
      for (size_t i = 0; i+1 < raw.size();) {
        size_t j = raw.size()-1;
        while (j > i+1 && !clear_segment(g,blocked,raw[i],raw[j])) --j;
        result.push_back(raw[j]); i = j;
      }
      return result;
    }
    for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
      if (!dx && !dy) continue;
      const cv::Point v(u.x+dx,u.y+dy);
      if (!g.inside(v) || blocked(v)) continue;
      if (dx && dy && (blocked(u.y,v.x) || blocked(v.y,u.x))) continue;
      const int vi = index(v);
      const double cost = costs[ui]+std::hypot(dx,dy);
      if (cost < costs[vi]) {
        costs[vi] = cost; parents[vi] = ui;
        queue.emplace(cost+std::hypot(v.x-b.x,v.y-b.y),vi);
      }
    }
  }
  return {};
}
Route fixed_gate_route(const Config &cfg, const Gate &gate) {
  if (!gate.center.allFinite() || !gate.normal.allFinite() || gate.normal.norm()<1e-9) return {};
  const Vec2 normal=gate.normal.normalized();
  const Vec2 pre=gate.center-cfg.pre_distance*normal, post=gate.center+cfg.post_distance*normal;
  if (normal.x()<=0 || pre.x() < -1e-9 || pre.dot(normal) < -1e-9 || post.x()<=0) return {};
  return {Vec2::Zero(),pre,post};
}
std::string straight_segment_check(const Grid &g, const Layers &layers, const Vec2 &a, const Vec2 &b) {
  auto obstruction = [&](const Vec2 &p,const std::string &name) {
    const auto q = g.cell(p);
    if (!g.inside(q)) return name+" outside map";
    if (layers.obstacles(q)) return name+" blocked by obstacle inflation";
    if (layers.unknown(q)) return name+" blocked by unknown inflation";
    return std::string{};
  };
  for (const auto &entry : {std::make_pair(a,"Start"),std::make_pair(b,"Goal")}) {
    const auto reason=obstruction(entry.first,entry.second);
    if (!reason.empty()) return reason;
  }
  const int count=std::max(2,static_cast<int>(std::ceil((b-a).norm()/(g.cfg.resolution/2)))+1);
  for (int i=1;i<count-1;++i) {
    const auto reason=obstruction(a+(b-a)*(static_cast<double>(i)/(count-1)),"Segment");
    if (!reason.empty()) return reason;
  }
  return {};
}
Route plan(const Grid &g, const cv::Mat1b &blocked, const Gate &gate) {
  const auto route=fixed_gate_route(g.cfg,gate);
  for (size_t i=1;i<route.size();++i)
    if (!clear_segment(g,blocked,route[i-1],route[i])) return {};
  return route;
}
Trajectory trajectory(const Route &route, const Config &cfg) {
  cfg.validate();
  Trajectory rows;
  double elapsed = 0;
  for (size_t j = 1; j < route.size(); ++j) {
    const Vec2 delta = route[j]-route[j-1];
    const double distance = delta.norm();
    if (distance < 1e-8) continue;
    const double duration = std::max({1.875*distance/cfg.max_speed,
        std::sqrt((10/std::sqrt(3))*distance/cfg.max_acceleration),cfg.sample_dt});
    const int count = static_cast<int>(std::ceil(duration/cfg.sample_dt));
    for (int i = rows.empty() ? 0 : 1; i <= count; ++i) {
      const double u = static_cast<double>(i)/count, u2 = u*u, u3 = u2*u, u4 = u3*u, u5 = u4*u;
      Sample row;
      row.t = elapsed+u*duration;
      row.p.head<2>() = route[j-1]+(10*u3-15*u4+6*u5)*delta;
      row.v.head<2>() = (30*u2-60*u3+30*u4)/duration*delta;
      row.a.head<2>() = (60*u-180*u2+120*u3)/(duration*duration)*delta;
      rows.push_back(row);
    }
    elapsed += duration;
  }
  return rows;
}
Sample interpolate(const Sample &l, const Sample &r, double t) {
  const double d = r.t-l.t, u = (t-l.t)/d, u2 = u*u, u3 = u2*u, u4 = u3*u, u5 = u4*u;
  const Vec3 v0 = l.v*d, v1 = r.v*d, a0 = l.a*d*d, a1 = r.a*d*d;
  const Vec3 dp = r.p-l.p-v0-a0/2, dv = v1-v0-a0, da = a1-a0;
  const Vec3 c3 = 10*dp-4*dv+da/2, c4 = -15*dp+7*dv-da, c5 = 6*dp-3*dv+da/2;
  Sample result;
  result.t = t;
  result.p = l.p+v0*u+a0*u2/2+c3*u3+c4*u4+c5*u5;
  result.v = (v0+a0*u+3*c3*u2+4*c4*u3+5*c5*u4)/d;
  result.a = (a0+6*c3*u+12*c4*u2+20*c5*u3)/(d*d);
  return result;
}
Eigen::Matrix2d basis(double heading) {
  Eigen::Matrix2d r;
  r << std::cos(heading),-std::sin(heading),std::sin(heading),std::cos(heading);
  return r;
}
Vec2 world_xy(const Vec2 &local, const Vec3 &reference, double heading) {
  return reference.head<2>()+basis(heading)*local;
}
Vec2 grid_origin(const Config &cfg, const Vec3 &reference, double heading) {
  return world_xy({cfg.x_min,cfg.y_min},reference,heading);
}
std::vector<int8_t> raw_map(const Grid &g) {
  std::vector<int8_t> result;
  result.reserve(g.occupied.total());
  for (int y = 0; y < g.occupied.rows; ++y) for (int x = 0; x < g.occupied.cols; ++x)
    result.push_back(g.occupied(y,x) ? 100 : (g.observed(y,x) ? 0 : -1));
  return result;
}
}  // namespace door_navigation
