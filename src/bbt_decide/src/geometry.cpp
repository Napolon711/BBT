#include "bbt_decide/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace bbt_decide
{
namespace
{
constexpr double kEpsilon = 1e-8;

bool finite(const Vec2& p) { return std::isfinite(p.x) && std::isfinite(p.y); }
double cross(const Vec2& a, const Vec2& b, const Vec2& c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}
bool on_segment(const Vec2& p, const Vec2& a, const Vec2& b)
{
  return std::abs(cross(a, b, p)) <= kEpsilon * std::max(1.0, std::hypot(b.x-a.x, b.y-a.y)) &&
         p.x >= std::min(a.x, b.x) - kEpsilon && p.x <= std::max(a.x, b.x) + kEpsilon &&
         p.y >= std::min(a.y, b.y) - kEpsilon && p.y <= std::max(a.y, b.y) + kEpsilon;
}

// -1 outside, 0 boundary, 1 strictly inside. Ray casting also handles concavity.
int containment(const std::vector<Vec2>& polygon, const Vec2& p)
{
  if (!finite(p) || polygon.size() < 3) return -1;
  bool inside = false;
  for (size_t i = 0; i < polygon.size(); ++i)
  {
    const auto& a = polygon[i];
    const auto& b = polygon[(i + 1) % polygon.size()];
    if (on_segment(p, a, b)) return 0;
    if ((a.y > p.y) != (b.y > p.y) &&
        p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
  }
  return inside ? 1 : -1;
}

bool intersects(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d)
{
  const double ac = cross(a, b, c), ad = cross(a, b, d);
  const double ca = cross(c, d, a), cb = cross(c, d, b);
  if (((ac > kEpsilon && ad < -kEpsilon) || (ac < -kEpsilon && ad > kEpsilon)) &&
      ((ca > kEpsilon && cb < -kEpsilon) || (ca < -kEpsilon && cb > kEpsilon))) return true;
  return on_segment(a, c, d) || on_segment(b, c, d) || on_segment(c, a, b) || on_segment(d, a, b);
}

bool valid_polygon(const std::vector<Vec2>& polygon)
{
  if (polygon.size() < 3) return false;
  long double area2 = 0;
  for (size_t i = 0; i < polygon.size(); ++i)
  {
    const auto& a = polygon[i];
    const auto& b = polygon[(i + 1) % polygon.size()];
    if (!finite(a) || std::hypot(a.x-b.x, a.y-b.y) <= kEpsilon) return false;
    area2 += static_cast<long double>(a.x) * b.y - static_cast<long double>(a.y) * b.x;
    for (size_t j = i + 1; j < polygon.size(); ++j)
    {
      if (j == i + 1 || (i == 0 && j + 1 == polygon.size())) continue;
      if (intersects(a, b, polygon[j], polygon[(j + 1) % polygon.size()])) return false;
    }
  }
  return std::isfinite(area2) && std::abs(area2) > kEpsilon;
}

void collect_segment_cuts(const std::vector<Vec2>& polygon, const Vec2& a,
                          const Vec2& b, std::vector<double>& cuts)
{
  const Vec2 r{b.x-a.x, b.y-a.y};
  const double length2 = r.x*r.x+r.y*r.y;
  if (length2 == 0) return;
  for (size_t i = 0; i < polygon.size(); ++i)
  {
    const auto& c = polygon[i];
    const auto& d = polygon[(i + 1) % polygon.size()];
    const Vec2 s{d.x-c.x, d.y-c.y};
    const double denominator = r.x*s.y-r.y*s.x;
    if (std::abs(denominator) < 1e-12)
    {
      // A path along a cell boundary must still split at the boundary ends;
      // otherwise an uncovered collinear gap could be skipped as one interval.
      if (std::abs(cross(a, b, c)) <= kEpsilon)
        for (const auto& p : {c, d})
        {
          const double t = ((p.x-a.x)*r.x+(p.y-a.y)*r.y) / length2;
          if (t >= 0 && t <= 1) cuts.push_back(t);
        }
      continue;
    }
    const double t = ((c.x-a.x)*s.y-(c.y-a.y)*s.x) / denominator;
    const double u = ((c.x-a.x)*r.y-(c.y-a.y)*r.x) / denominator;
    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) cuts.push_back(t);
  }
}

// Split a segment at all polygon edge intersections, then inspect each open
// interval. This catches sparse paths crossing a concave polygon and permits a
// normal approach that terminates exactly at its entrance boundary.
bool segment_enters_polygon(const std::vector<Vec2>& polygon, const Vec2& a, const Vec2& b)
{
  if (polygon.size() < 3) return false;
  if (containment(polygon, a) == 1 || containment(polygon, b) == 1) return true;
  std::vector<double> cuts{0.0, 1.0};
  collect_segment_cuts(polygon, a, b, cuts);
  const Vec2 r{b.x-a.x, b.y-a.y};
  std::sort(cuts.begin(), cuts.end());
  for (size_t i = 1; i < cuts.size(); ++i)
  {
    const double t = (cuts[i-1]+cuts[i]) / 2.0;
    if (containment(polygon, {a.x+t*r.x, a.y+t*r.y}) == 1) return true;
  }
  return false;
}

bool interiors_overlap(const std::vector<Vec2>& a, const std::vector<Vec2>& b)
{
  for (const auto& p : a) if (containment(b, p) == 1) return true;
  for (size_t i = 0; i < a.size(); ++i)
  {
    const auto& p = a[i];
    const auto& q = a[(i+1) % a.size()];
    if (segment_enters_polygon(b, p, q)) return true;
    // Coincident boundaries have no strict edge intersections. Every simple
    // polygon has an ear; its centroid is an interior sample without guessing
    // an offset that could jump over a narrow cell.
    const auto& previous = a[(i+a.size()-1) % a.size()];
    const Vec2 sample{(previous.x+p.x+q.x)/3, (previous.y+p.y+q.y)/3};
    if (containment(a, sample) == 1 && containment(b, sample) == 1) return true;
  }
  return false;
}

int index_of(const Config& cfg, const std::string& name)
{
  const auto it = std::find(cfg.regions.begin(), cfg.regions.end(), name);
  return it == cfg.regions.end() ? -1 : static_cast<int>(it-cfg.regions.begin());
}
}  // namespace

int region_for_position(const Config& cfg, const Vec2& position)
{
  if (!finite(position)) return -1;
  int result = -1;
  for (const auto& cell : cfg.region_geometry)
  {
    if (containment(cell.polygon, position) < 0) continue;
    const int index = index_of(cfg, cell.name);
    if (index < 0 || (result >= 0 && result != index)) return -1;
    result = index;
  }
  return result;
}

bool segment_within_region(const Config& cfg, const Vec2& start, const Vec2& end, int region)
{
  if (!finite(start) || !finite(end) || region < 0 ||
      region >= static_cast<int>(cfg.regions.size())) return false;
  const auto covered = [&cfg, region](const Vec2& p) {
    for (const auto& cell : cfg.region_geometry)
      if (cell.name == cfg.regions[region] && containment(cell.polygon, p) >= 0) return true;
    return false;
  };
  if (!covered(start) || !covered(end)) return false;
  std::vector<double> cuts{0.0, 1.0};
  for (const auto& cell : cfg.region_geometry)
    if (cell.name == cfg.regions[region]) collect_segment_cuts(cell.polygon, start, end, cuts);
  std::sort(cuts.begin(), cuts.end());
  for (size_t i = 1; i < cuts.size(); ++i)
  {
    if (cuts[i] == cuts[i-1]) continue;
    const double t = (cuts[i-1]+cuts[i]) / 2.0;
    if (!covered({start.x + t*(end.x-start.x), start.y + t*(end.y-start.y)})) return false;
  }
  return true;
}

bool point_inside_tunnel(const Config& cfg, const Vec2& position)
{
  if (!cfg.enable_tunnel) return false;
  for (const auto& tunnel : cfg.tunnels)
    if (containment(tunnel.corners, position) == 1) return true;
  return false;
}

bool segment_crosses_tunnel(const std::vector<TunnelDef>& tunnels, const Vec2& start, const Vec2& end)
{
  if (!finite(start) || !finite(end)) return true;
  for (const auto& tunnel : tunnels)
    if (segment_enters_polygon(tunnel.corners, start, end)) return true;
  return false;
}

bool segment_crosses_tunnel(const Config& cfg, const Vec2& start, const Vec2& end)
{
  return cfg.enable_tunnel && segment_crosses_tunnel(cfg.tunnels, start, end);
}

bool path_clears_tunnel_exit(const TunnelDef& tunnel, int exit_side,
                            const std::vector<Vec2>& positions)
{
  if ((exit_side != 0 && exit_side != 1) || positions.size() < 2 ||
      !valid_polygon(tunnel.corners) || !finite(tunnel.align[0]) ||
      !finite(tunnel.align[1])) return false;
  for (const auto& point : positions) if (!finite(point)) return false;

  const auto& exit = tunnel.align[exit_side];
  const auto& entry = tunnel.align[1-exit_side];
  const double length = std::hypot(exit.x-entry.x, exit.y-entry.y);
  if (!std::isfinite(length) || length <= kEpsilon) return false;
  const Vec2 axis{(exit.x-entry.x)/length, (exit.y-entry.y)/length};
  const Vec2 center = tunnel.has_center ? tunnel.center :
      Vec2{entry.x+(exit.x-entry.x)/2, entry.y+(exit.y-entry.y)/2};
  if (!finite(center) || containment(tunnel.corners, center) < 0 ||
      containment(tunnel.corners, entry) == 1 ||
      containment(tunnel.corners, exit) == 1) return false;
  const auto projection = [&axis, &exit](const Vec2& p) {
    return (p.x-exit.x)*axis.x + (p.y-exit.y)*axis.y;
  };
  const double center_projection = projection(center);
  double previous_projection = projection(positions.front());
  if (!std::isfinite(center_projection) || !std::isfinite(previous_projection) ||
      previous_projection < center_projection-kEpsilon ||
      containment(tunnel.corners, positions.front()) != 1) return false;

  bool left_tunnel = false;
  const auto inspect = [&](const Vec2& point) {
    const int location = containment(tunnel.corners, point);
    const double along = projection(point);
    if (!std::isfinite(along)) return false;
    if (left_tunnel) return location != 1;
    if (along < previous_projection-kEpsilon) return false;
    previous_projection = along;
    if (location == 1) return true;
    // Inspect the first boundary contact, not just an outside path sample:
    // a diagonal side exit must not be mistaken for crossing the exit plane.
    if (std::abs(along) > kEpsilon) return false;
    left_tunnel = true;
    return true;
  };
  for (size_t i = 1; i < positions.size(); ++i)
  {
    const auto& a = positions[i-1];
    const auto& b = positions[i];
    std::vector<double> cuts{0.0, 1.0};
    collect_segment_cuts(tunnel.corners, a, b, cuts);
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
    const auto at = [&a, &b](double t) {
      return Vec2{a.x+t*(b.x-a.x), a.y+t*(b.y-a.y)};
    };
    for (size_t cut = 1; cut < cuts.size(); ++cut)
      if (!inspect(at((cuts[cut-1]+cuts[cut])/2)) || !inspect(at(cuts[cut])))
        return false;
  }
  return left_tunnel && containment(tunnel.corners, positions.back()) != 1;
}

bool validate_geometry(const Config& cfg, std::string& error)
{
  for (const auto& tunnel : cfg.tunnels)
    if (!tunnel.corners.empty() && !valid_polygon(tunnel.corners))
    { error = "洞 " + tunnel.name + " 的 corners 必须是有限、非退化、不自交多边形"; return false; }
  if (cfg.region_geometry.empty()) return true;
  std::set<std::string> covered;
  for (size_t i = 0; i < cfg.region_geometry.size(); ++i)
  {
    const auto& cell = cfg.region_geometry[i];
    if (index_of(cfg, cell.name) < 0 || !valid_polygon(cell.polygon))
    { error = "region_geometry: " + cell.name + " 必须引用已知区域并提供有限、非退化、不自交多边形"; return false; }
    covered.insert(cell.name);
    for (size_t j = 0; j < i; ++j)
      if (cell.name != cfg.region_geometry[j].name &&
          (interiors_overlap(cell.polygon, cfg.region_geometry[j].polygon) ||
           interiors_overlap(cfg.region_geometry[j].polygon, cell.polygon)))
      { error = "region_geometry: 区域内部重叠: " + cell.name + " / " + cfg.region_geometry[j].name; return false; }
  }
  if (covered.size() != cfg.regions.size())
  { error = "region_geometry: 每个region都必须有几何范围"; return false; }
  for (const auto& point : cfg.points)
    if (region_for_position(cfg, point.pos) != point.region || point_inside_tunnel(cfg, point.pos))
    { error = "点 " + std::to_string(point.id) + " 的region与几何不一致或位于洞内"; return false; }
  for (const auto& tunnel : cfg.tunnels)
  {
    if (tunnel.corners.empty())
    { error = "洞 " + tunnel.name + " 在几何配置下必须提供corners"; return false; }
    for (int side = 0; side < 2; ++side)
      if (region_for_position(cfg, tunnel.align[side]) != tunnel.align_region[side] ||
          containment(tunnel.corners, tunnel.align[side]) == 1)
      { error = "洞 " + tunnel.name + " 的align区域与几何不一致"; return false; }
    if (!segment_enters_polygon(tunnel.corners, tunnel.align[0], tunnel.align[1]) ||
        (tunnel.has_center && containment(tunnel.corners, tunnel.center) < 0))
    { error = "洞 " + tunnel.name + " 的center/align与洞体几何不一致"; return false; }
  }
  return true;
}
}  // namespace bbt_decide
