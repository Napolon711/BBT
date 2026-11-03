#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <unistd.h>
#include "bbt_decide/geometry.hpp"
#include "bbt_decide/planner.hpp"

using namespace bbt_decide;
#define CHECK(x) do { if (!(x)) throw std::runtime_error("line " + std::to_string(__LINE__) + ": " #x); } while (0)

namespace
{
Config load(const std::string& yaml, bool expect_success = true)
{
  char name[] = "/tmp/bbt_geometry_XXXXXX";
  const int fd = mkstemp(name);
  CHECK(fd >= 0);
  close(fd);
  { std::ofstream out(name); out << yaml; }
  Config result;
  std::string error;
  std::vector<std::string> warnings;
  const bool ok = load_config(name, result, error, warnings);
  std::remove(name);
  if (ok != expect_success) throw std::runtime_error("configuration result: " + error);
  return result;
}

const std::string base =
    "geometry_reference: none\nregions: [left, right]\n"
    "region_geometry:\n"
    "  - {name: left, polygon: [[0, 0], [10, 0], [10, 10], [0, 10]]}\n"
    "  - {name: right, polygon: [[10, 0], [20, 0], [20, 10], [10, 10]]}\n"
    "points: [{id: 1, x: 1, y: 5, zone: home, region: left}]\n"
    "tunnels:\n"
    "  - name: hole\n"
    "    corners: [[9, 4.5], [11, 4.5], [11, 5.5], [9, 5.5]]\n"
    "    align: [{p: [9, 5], region: left}, {p: [11, 5], region: right}]\n"
    "    center: [10, 5]\n";

void arbitrary_coordinates_and_tunnel_interior()
{
  const auto cfg = load(base);
  CHECK(region_for_position(cfg, {7.23, 2.84}) == 0);
  CHECK(region_for_position(cfg, {17.23, 8.84}) == 1);
  CHECK(region_for_position(cfg, {10, 2}) == -1);  // shared boundary is ambiguous
  CHECK(region_for_position(cfg, {-0.01, 2}) == -1);
  CHECK(region_for_position(cfg, {NAN, 2}) == -1);
  CHECK(point_inside_tunnel(cfg, {10, 5}));
  CHECK(!point_inside_tunnel(cfg, {9, 5}));  // entry is an admissible stopping point
  CHECK(!point_inside_tunnel(cfg, {8, 5}));
  CHECK(segment_crosses_tunnel(cfg, {8, 5}, {12, 5}));
  CHECK(segment_crosses_tunnel(cfg, {10, 5}, {12, 5}));
  CHECK(!segment_crosses_tunnel(cfg, {8, 5}, {9, 5}));
  CHECK(!segment_crosses_tunnel(cfg, {8, 4.5}, {12, 4.5}));
  CHECK(!segment_crosses_tunnel(cfg, {8, 3}, {12, 3}));
  auto concave = cfg;
  concave.tunnels[0].corners = {{2,2},{4,2},{4,4},{3,4},{3,3},{2,3}};
  CHECK(segment_crosses_tunnel(concave, {1,2.5}, {5,2.5}));
  CHECK(!segment_crosses_tunnel(concave, {1,3.5}, {2.9,3.5}));
}

void malformed_geometry_is_rejected()
{
  for (const auto* polygon : {"[[0,0],[20,0],[20,10],[0,10]]",  // overlap
                             "[[0,0],[10,0],[10,10],[0,10]]", // identical
                             "[[0,0],[10,10],[0,10],[10,0]]",  // self intersection
                             "[[0,0],[1,1],[2,2]]",             // no area
                             "[[0,0],[.nan,1],[0,2]]"})
  {
    std::string text = base;
    const auto begin = text.find("[[10, 0], [20, 0], [20, 10], [10, 10]]");
    text.replace(begin, std::string("[[10, 0], [20, 0], [20, 10], [10, 10]]").size(), polygon);
    load(text, false);
  }
  std::string mismatch = base;
  mismatch.replace(mismatch.find("x: 1, y: 5"), std::string("x: 1, y: 5").size(), "x: 19, y: 5");
  load(mismatch, false);
  std::string wrong_gate = base;
  wrong_gate.replace(wrong_gate.find("p: [9, 5], region: left"),
                     std::string("p: [9, 5], region: left").size(), "p: [9, 5], region: right");
  load(wrong_gate, false);
  std::string wrong_center = base;
  wrong_center.replace(wrong_center.find("center: [10, 5]"),
                       std::string("center: [10, 5]").size(), "center: [1, 9]");
  load(wrong_center, false);
}

void same_region_cells_and_legacy_ids()
{
  const auto legacy = load("regions: [left]\npoints: [{id: 1, x: 1, y: 5, region: left}]\n");
  CHECK(region_for_position(legacy, {1,5}) == -1);
  auto cfg = load(base);
  cfg.region_geometry.push_back({"left", {{1,1},{2,1},{2,2},{1,2}}});
  std::string error;
  CHECK(validate_geometry(cfg, error));
  CHECK(region_for_position(cfg, {1.5,1.5}) == 0);
}

void navigation_segments_stay_in_the_selected_region()
{
  Config cfg;
  cfg.regions = {"route", "other"};
  cfg.region_geometry = {
      {"route", {{0,0},{4,0},{4,2},{0,2}}},
      {"route", {{4,0},{10,0},{10,2},{4,2}}}};
  CHECK(segment_within_region(cfg, {0.5,1}, {9.5,1}, 0));
  CHECK(segment_within_region(cfg, {0.5,0}, {9.5,0}, 0)); // boundary is in the closed union
  CHECK(!segment_within_region(cfg, {0.5,1}, {10.5,1}, 0));
  CHECK(!segment_within_region(cfg, {0.5,1}, {9.5,1}, 1));
  CHECK(!segment_within_region(cfg, {NAN,1}, {9.5,1}, 0));
  CHECK(!segment_within_region(cfg, {0.5,1}, {9.5,1}, -1));

  // Endpoints in the same region are insufficient: the full sparse path must
  // reject even a narrow uncovered/foreign strip between its endpoint samples.
  cfg.region_geometry[1].polygon = {{4.0001,0},{10,0},{10,2},{4.0001,2}};
  CHECK(!segment_within_region(cfg, {0.5,1}, {9.5,1}, 0));
  cfg.region_geometry.push_back({"other", {{4,0},{4.0001,0},{4.0001,2},{4,2}}});
  CHECK(!segment_within_region(cfg, {0.5,1}, {9.5,1}, 0));
  CHECK(!segment_within_region(cfg, {0.5,0}, {9.5,0}, 0));
  cfg.region_geometry.back().name = "route";
  CHECK(segment_within_region(cfg, {0.5,1}, {9.5,1}, 0));
  CHECK(segment_within_region(cfg, {0.5,0}, {9.5,0}, 0));

  cfg.region_geometry = {{"route", {{0,0},{4,0},{4,4},{3,4},{3,1},{1,1},{1,4},{0,4}}}};
  CHECK(!segment_within_region(cfg, {0.5,3}, {3.5,3}, 0)); // concave cutout
  CHECK(segment_within_region(cfg, {0.5,0.5}, {3.5,0.5}, 0));
}

void completed_traversal_only_clears_its_authorized_exit()
{
  TunnelDef tunnel;
  tunnel.corners = {{9.82,3.5},{8.82,3.5},{8.82,1},{9.82,1}};
  tunnel.align[0] = {9.32,3.5};
  tunnel.align[1] = {9.32,1};
  tunnel.center = {9.32,2.15};
  tunnel.has_center = true;
  const Vec2 actual{9.3200002936,1.0202299563};
  CHECK(path_clears_tunnel_exit(tunnel, 1,
      {actual, actual, {9.32,.45}, {11.37,.66}}));
  CHECK(path_clears_tunnel_exit(tunnel, 1, {{9.32,2.15},{9.32,.45}}));
  CHECK(path_clears_tunnel_exit(tunnel, 0, {{9.32,3.3},{9.32,4},{8,4}}));
  CHECK(!path_clears_tunnel_exit(tunnel, 1, {{9.32,2.2},{9.32,.45}}));
  CHECK(!path_clears_tunnel_exit(tunnel, 1, {actual,{9.32,1.1},{9.32,.45}}));
  CHECK(!path_clears_tunnel_exit(tunnel, 1, {actual,{10.5,1.01},{9.32,.45}}));
  CHECK(!path_clears_tunnel_exit(tunnel, 1, {actual,{9.32,.45},actual,{11.37,.66}}));
  CHECK(!path_clears_tunnel_exit(tunnel, 1, {actual,{9.32,.45},{9.32,4}}));
  CHECK(!path_clears_tunnel_exit(tunnel, 0, {actual,{9.32,.45}}));
  CHECK(path_clears_tunnel_exit(tunnel, 1, {actual,{9.32,1}})); // raw final may be the exit boundary
  CHECK(!path_clears_tunnel_exit(tunnel, 1, {{9.32,1},{9.32,.45}})); // start on boundary
  CHECK(!path_clears_tunnel_exit(tunnel, 1, {{9.32,.9},{9.32,.45}}));
  CHECK(!path_clears_tunnel_exit(tunnel, 1, {actual}));
  CHECK(!path_clears_tunnel_exit(tunnel, 1, {}));
  CHECK(!path_clears_tunnel_exit(tunnel, -1, {actual,{9.32,.45}}));
  CHECK(!path_clears_tunnel_exit(tunnel, 2, {actual,{9.32,.45}}));
  CHECK(!path_clears_tunnel_exit(tunnel, 1, {actual,{NAN,.45}}));

  auto no_center = tunnel;
  no_center.has_center = false;
  CHECK(!path_clears_tunnel_exit(no_center, 1, {{9.32,2.3},{9.32,.45}}));
  CHECK(path_clears_tunnel_exit(no_center, 1, {{9.32,2.2},{9.32,.45}}));
  for (int malformed = 0; malformed < 6; ++malformed)
  {
    auto invalid = tunnel;
    if (malformed == 0) invalid.align[0] = invalid.align[1];
    if (malformed == 1) invalid.align[1].x = NAN;
    if (malformed == 2) invalid.center.y = NAN;
    if (malformed == 3) invalid.corners = {{0,0},{1,1},{2,2}};
    if (malformed == 4) invalid.center = {0,0};
    if (malformed == 5) invalid.align[1] = {9.32,1.01};
    CHECK(!path_clears_tunnel_exit(invalid, 1, {actual,{9.32,.45}}));
  }

  // A sparse segment can leave a concave tunnel and re-enter before its final
  // sample. Endpoints alone must not authorize that hidden side excursion.
  TunnelDef concave;
  concave.corners = {{0,0},{4,0},{4,1},{2,1},{2,2},{4,2},{4,3},{0,3}};
  concave.align[0] = {0,.5};
  concave.align[1] = {4,.5};
  concave.center = {1,.5};
  concave.has_center = true;
  CHECK(!path_clears_tunnel_exit(concave, 1, {{3,.5},{3,2.5},{5,2.5}}));
  concave.align[0] = {0,1.5};
  concave.align[1] = {2,1.5};
  concave.center = {.5,1.5};
  CHECK(path_clears_tunnel_exit(concave, 1, {{1.5,1.5},{2.5,1.5}}));
  CHECK(!path_clears_tunnel_exit(concave, 1, {{1.5,1.5},{5,2.5}}));

  // Clearance is invariant under the same rigid map transform as the tunnel.
  const FrameTransform transform{true,3.25,-4.75,37};
  for (auto& corner : tunnel.corners) corner = apply_frame_transform(corner, transform);
  for (auto& align : tunnel.align) align = apply_frame_transform(align, transform);
  tunnel.center = apply_frame_transform(tunnel.center, transform);
  std::vector<Vec2> transformed{actual,actual,{9.32,.45},{11.37,.66}};
  for (auto& position : transformed) position = apply_frame_transform(position, transform);
  CHECK(path_clears_tunnel_exit(tunnel, 1, transformed));
}

void frame_and_team_transform_every_geometry()
{
  auto cfg = load(base + "field_to_map: {enabled: true, dx: 3, dy: -4, dyaw_deg: 90}\n");
  CHECK(region_for_position(cfg, apply_frame_transform({17, 7}, cfg.field_to_map)) == 1);
  CHECK(point_inside_tunnel(cfg, apply_frame_transform({10, 5}, cfg.field_to_map)));
  cfg.geometry_reference = "red";
  configure_own_team(cfg, "blue");
  CHECK(region_for_position(cfg, apply_frame_transform({11, 8}, cfg.field_to_map)) == 1);
  CHECK(point_inside_tunnel(cfg, apply_frame_transform({18, 10}, cfg.field_to_map)));
}

void shipped_geometry_loads()
{
  for (const auto* name : {"decision.yaml", "corridor_demo.yaml", "plain_demo.yaml"})
  {
    Config cfg;
    std::string error;
    std::vector<std::string> warnings;
    CHECK(load_config(std::string(SRC_DIR) + "/config/" + name, cfg, error, warnings));
    CHECK(!cfg.region_geometry.empty());
    for (const auto& point : cfg.points) CHECK(region_for_position(cfg, point.pos) == point.region);
    configure_own_team(cfg, "blue");
    CHECK(validate_geometry(cfg, error));
  }
}

void road_to_central_ground_seams_remain_in_mid()
{
  for (const auto* team : {"red", "blue"})
  {
    Config cfg;
    std::string error;
    std::vector<std::string> warnings;
    CHECK(load_config(std::string(SRC_DIR) + "/config/decision.yaml", cfg, error, warnings));
    configure_own_team(cfg, team);
    const auto deployed = [team](Vec2 point) {
      return std::string(team) == "blue" ? Vec2{28-point.x, 15-point.y} : point;
    };
    // Captured post-exit plan from the stalled demo. The road and the central
    // slope differ by 4 mm at their seam; that ground must keep the mid label.
    const std::vector<Vec2> path{{9.319998042489422, 1.0210346013193874},
        {9.32, .45}, {11.716, 2.86}, {12.253446250469228, 6.771680015377857}, {15.69, 10.9}};
    const int mid = region_for_position(cfg, deployed(path.back()));
    CHECK(mid >= 0 && cfg.regions[mid] == "mid");
    for (size_t i = 1; i < path.size(); ++i)
      CHECK(segment_within_region(cfg, deployed(path[i-1]), deployed(path[i]), mid));
    CHECK(region_for_position(cfg, deployed({10.95, 2.092})) == mid);
    CHECK(region_for_position(cfg, deployed({17.05, 12.908})) == mid);
    CHECK(segment_within_region(cfg, deployed({17.05, 12.90}), deployed({17.05, 12.915}), mid));
    // The neighboring low region, its shared boundary and all tunnel bodies
    // retain their existing meaning; this is not a general cross-region permit.
    const int red_low = region_for_position(cfg, deployed({13.5, 2.085}));
    const int blue_low = region_for_position(cfg, deployed({14.5, 12.915}));
    CHECK(red_low >= 0 && cfg.regions[red_low] == "own_low");
    CHECK(blue_low >= 0 && cfg.regions[blue_low] == "enemy_low");
    CHECK(region_for_position(cfg, deployed({13.5, 2.09})) == -1);
    CHECK(!segment_within_region(cfg, deployed({13.5, 2.07}), deployed({13.5, 2.10}), mid));
    CHECK(segment_crosses_tunnel(cfg, deployed({9.32, .45}), deployed({9.32, 3.6})));
    CHECK(validate_geometry(cfg, error));
  }
}

void production_points_clear_finite_tunnel_segments()
{
  std::ifstream input(std::string(SRC_DIR) + "/config/decision.yaml");
  CHECK(input.good());
  const std::string original((std::istreambuf_iterator<char>(input)),
                             std::istreambuf_iterator<char>());
  const Config baseline = load(original);
  CHECK(baseline.points.size() == 39);
  CHECK(baseline.tunnels.size() == 6);
  CHECK(std::abs(baseline.gate_clear_m - 0.5) < 1e-12);

  const auto nearest_to_tunnel = [](const Config& cfg, const std::string& name) {
    const TunnelDef* tunnel = nullptr;
    for (const auto& candidate : cfg.tunnels)
      if (candidate.name == name) tunnel = &candidate;
    CHECK(tunnel != nullptr);
    const PointDef* nearest = nullptr;
    double nearest_distance = INFINITY;
    for (const auto& point : cfg.points)
    {
      const double distance = dist_to_segment(point.pos, tunnel->align[0], tunnel->align[1]);
      if (distance < nearest_distance)
      {
        nearest_distance = distance;
        nearest = &point;
      }
    }
    CHECK(nearest != nullptr);
    return std::make_pair(nearest->id, nearest_distance);
  };

  const auto own_road = nearest_to_tunnel(baseline, "own_road_tunnel");
  CHECK(own_road.first == 47);
  CHECK(std::abs(own_road.second - 1.5024313628249375) < 1e-9);
  const auto enemy_road = nearest_to_tunnel(baseline, "enemy_road_tunnel");
  CHECK(enemy_road.first == 153);
  CHECK(std::abs(enemy_road.second - 0.9148223871331518) < 1e-9);

  // Exercise the actual loader's rigid transform, then both deployment teams.
  // The original tactical table, IDs, region bindings and centres stay intact.
  std::string transformed = original;
  const auto transform_begin = transformed.find("\nfield_to_map:\n");
  const auto transform_end = transformed.find("\nsafety:\n", transform_begin);
  CHECK(transform_begin != std::string::npos && transform_end != std::string::npos);
  transformed.replace(transform_begin, transform_end-transform_begin,
      "\nfield_to_map:\n  enabled: true\n  dx: 3.25\n  dy: -4.75\n  dyaw_deg: 37\n");
  for (const auto& yaml : {original, transformed})
    for (const auto* team : {"red", "blue"})
    {
      auto cfg = load(yaml);
      configure_own_team(cfg, team);
      CHECK(cfg.points.size() == baseline.points.size());
      for (size_t p = 0; p < cfg.points.size(); ++p)
      {
        CHECK(cfg.points[p].id == baseline.points[p].id);
        CHECK(cfg.points[p].region == baseline.points[p].region);
        CHECK(!too_close_to_gate(cfg, cfg.points[p].pos));
        for (int from = 0; from < static_cast<int>(cfg.regions.size()); ++from)
          CHECK(point_usable(cfg, from, cfg.points[p]));
        for (size_t t = 0; t < cfg.tunnels.size(); ++t)
        {
          const double expected = dist_to_segment(baseline.points[p].pos,
              baseline.tunnels[t].align[0], baseline.tunnels[t].align[1]);
          const double actual = dist_to_segment(cfg.points[p].pos,
              cfg.tunnels[t].align[0], cfg.tunnels[t].align[1]);
          CHECK(actual >= cfg.gate_clear_m);
          CHECK(std::abs(actual-expected) < 1e-9);
        }
      }
      CHECK(nearest_to_tunnel(cfg, "own_road_tunnel").first == 47);
      CHECK(nearest_to_tunnel(cfg, "enemy_road_tunnel").first == 153);
    }
}

void patrol_point_configuration()
{
  const std::string legacy =
      "enable_tunnel: false\n"
      "points: [{id: 1, x: 1, y: 5, zone: home}, "
      "{id: 28, x: 2, y: 5, zone: patrol}, "
      "{id: 40, x: 3, y: 5, zone: patrol}, "
      "{id: 109, x: 4, y: 5, zone: outpost}]\n";
  CHECK(load(legacy).patrol_point_ids.empty());
  CHECK(load(legacy + "patrol_point_ids: []\n").patrol_point_ids.empty());
  const auto selected = load(legacy + "patrol_point_ids: [40, 28]\n");
  CHECK(selected.patrol_point_ids == std::vector<int>({40, 28}));
  CHECK(selected.points.size() == 4);
  CHECK(find_point(selected, 109)->zone == kZoneOutpost);
  for (const auto* value : {"[999]", "[1]", "[109]", "[28, 28]", "28", "[bad]"})
    load(legacy + "patrol_point_ids: " + value + "\n", false);

  Config production;
  std::string error;
  std::vector<std::string> warnings;
  CHECK(load_config(std::string(SRC_DIR) + "/config/decision.yaml", production, error, warnings));
  CHECK(production.patrol_point_ids == std::vector<int>({28, 40, 70, 101}));
  CHECK(production.points.size() == 39);
  CHECK(find_point(production, 109)->zone == kZoneOutpost);
  CHECK(find_point(production, 114)->zone == kZoneOutpost);
  for (const auto* demo : {"corridor_demo.yaml", "plain_demo.yaml"})
  {
    Config cfg;
    CHECK(load_config(std::string(SRC_DIR) + "/config/" + demo, cfg, error, warnings));
    CHECK(cfg.patrol_point_ids.empty());
  }
}
}  // namespace

int main()
{
  arbitrary_coordinates_and_tunnel_interior();
  malformed_geometry_is_rejected();
  same_region_cells_and_legacy_ids();
  navigation_segments_stay_in_the_selected_region();
  completed_traversal_only_clears_its_authorized_exit();
  frame_and_team_transform_every_geometry();
  shipped_geometry_loads();
  road_to_central_ground_seams_remain_in_mid();
  production_points_clear_finite_tunnel_segments();
  patrol_point_configuration();
  std::cout << "geometry tests passed\n";
}
