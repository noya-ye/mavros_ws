#include "door_navigation/cloud.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <unordered_set>
#include <opencv2/imgproc.hpp>

namespace door_navigation {
namespace {
struct Voxel {
  int64_t x,y,z;
  bool operator==(const Voxel &v) const { return x==v.x && y==v.y && z==v.z; }
};
struct Hash {
  size_t operator()(const Voxel &v) const {
    size_t seed=std::hash<int64_t>{}(v.x);
    for (auto n : {v.y,v.z}) seed^=std::hash<int64_t>{}(n)+0x9e3779b9+(seed<<6)+(seed>>2);
    return seed;
  }
};
template<class T> double scalar(const uint8_t *p, bool swap) {
  std::array<uint8_t,sizeof(T)> bytes;
  std::copy_n(p,sizeof(T),bytes.begin());
  if (swap) std::reverse(bytes.begin(),bytes.end());
  T value; std::memcpy(&value,bytes.data(),sizeof(T)); return value;
}
}
std::vector<Vec3> read_cloud(const CloudView &c, const Vec3 &body, double self_radius) {
  if (!c.width || !c.height) return {};
  if (!c.data || !c.point_step || uint64_t(c.width)*c.point_step > c.row_step ||
      uint64_t(c.height-1)*c.row_step+uint64_t(c.width)*c.point_step > c.size)
    throw std::invalid_argument("Malformed PointCloud2 strides or data length");
  for (const auto &f : c.xyz) {
    if (f.datatype!=7 && f.datatype!=8) throw std::invalid_argument("XYZ fields must be FLOAT32 or FLOAT64");
    if (uint64_t(f.offset)+(f.datatype==7?4:8)>c.point_step)
      throw std::invalid_argument("XYZ field exceeds point_step");
  }
  const uint16_t one=1;
  const bool host_big=*reinterpret_cast<const uint8_t *>(&one)==0, swap=host_big!=c.big_endian;
  std::vector<Vec3> points;
  const size_t count=size_t(c.width)*c.height;
  points.reserve(count);
  std::unordered_set<Voxel,Hash> seen; seen.reserve(count);
  for (uint32_t y=0; y<c.height; ++y) for (uint32_t x=0; x<c.width; ++x) {
    const uint8_t *p=c.data+size_t(y)*c.row_step+size_t(x)*c.point_step;
    Vec3 v;
    for (int i=0; i<3; ++i) {
      const auto &f=c.xyz[i];
      v[i]=f.datatype==7 ? scalar<float>(p+f.offset,swap) : scalar<double>(p+f.offset,swap);
    }
    // Bounds protect float-to-integer voxel/raster conversion on malformed input.
    if (!v.allFinite() || v.cwiseAbs().maxCoeff()>1e6 || (v-body).norm()<=self_radius) continue;
    const Voxel key{static_cast<int64_t>(std::floor(v.x()/0.02)),
                    static_cast<int64_t>(std::floor(v.y()/0.02)),
                    static_cast<int64_t>(std::floor(v.z()/0.02))};
    if (seen.insert(key).second) points.push_back(v);
  }
  return points;
}
void observe_scan(Grid &grid, const Route &points, const Vec2 &origin) {
  struct Ray { double angle, range; };
  std::vector<Ray> rays; rays.reserve(points.size());
  for (const auto &p : points) {
    grid.add(p,origin);
    const Vec2 d=p-origin;
    if (d.norm()>grid.cfg.resolution) rays.push_back({std::atan2(d.y(),d.x()),d.norm()});
  }
  std::sort(rays.begin(),rays.end(),[](const Ray &a,const Ray &b) { return a.angle<b.angle; });
  for (size_t i=1; i<rays.size(); ++i) {
    const auto &a=rays[i-1],&b=rays[i];
    const double gap=b.angle-a.angle, range=std::min(a.range,b.range);
    // Interpolation is at most 3 degrees and 8 cm wide at the shorter return.
    // This is sampling continuity, not evidence of free space behind that return.
    if (gap>3*pi/180 || range*gap>0.08) continue;
    const cv::Point triangle[]{grid.cell(origin),
      grid.cell(origin+range*Vec2(std::cos(a.angle),std::sin(a.angle))),
      grid.cell(origin+range*Vec2(std::cos(b.angle),std::sin(b.angle)))};
    cv::fillConvexPoly(grid.observed,triangle,3,cv::Scalar(1));
  }
}
void complete_observation(Grid &grid) {
  cv::morphologyEx(grid.observed,grid.observed,cv::MORPH_CLOSE,cv::Mat1b::ones(3,3),
                  {-1,-1},1,cv::BORDER_CONSTANT,0);
}
}  // namespace door_navigation
