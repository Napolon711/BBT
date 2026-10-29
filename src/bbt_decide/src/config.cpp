// BBT 配置加载：把 decision.yaml 解析进 Config。
// 区域表 / 点 / 洞坐标全部只从这里进代码——换地图、换点、挪洞改 yaml 即可。
// field_to_map 在加载末尾生效：点表按场地系（图纸/STL）填，加载后统一搬到
// 导航建图系（tools/align_map.py 解算变换参数，与主工作区同款流程）。
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#include "bbt_decide/types.hpp"

namespace bbt_decide
{
namespace
{
// 读一个数值字段（存在且可转 double 才写进 out）。
void read_double(const YAML::Node& node, const char* key, double& out)
{
  if (node[key]) out = node[key].as<double>();
}
// 读一个整数字段。
void read_int(const YAML::Node& node, const char* key, int& out)
{
  if (node[key]) out = node[key].as<int>();
}
// 读一个字符串字段。
void read_str(const YAML::Node& node, const char* key, std::string& out)
{
  if (node[key]) out = node[key].as<std::string>();
}

// 读一张话题表（只覆盖写了的条目，其余保持默认）。
void read_topics(const YAML::Node& node, TopicMap& t)
{
  if (!node || !node.IsMap()) return;
  read_str(node, "nav_goal", t.nav_goal);
  read_str(node, "nav_status", t.nav_status);
  read_str(node, "nav_cancel", t.nav_cancel);
  read_str(node, "odom", t.odom);
  read_str(node, "cmd_vel", t.cmd_vel);
  read_str(node, "chassis_mode", t.chassis_mode);
  read_str(node, "spin", t.spin);
  read_str(node, "fold_cmd", t.fold_cmd);
  read_str(node, "fold_state", t.fold_state);
  read_str(node, "referee", t.referee);
  read_str(node, "op_cmd", t.op_cmd);
  read_str(node, "buy_ammo", t.buy_ammo);
  read_str(node, "buy_hp", t.buy_hp);
  read_str(node, "buy_revive", t.buy_revive);
  read_str(node, "data_source", t.data_source);
  read_str(node, "iface_status", t.iface_status);
}

// zone 字符串 → Zone 枚举；不认识返回 -1。
int zone_from_str(const std::string& s)
{
  if (s == "home") return kZoneHome;
  if (s == "patrol") return kZonePatrol;
  return -1;
}

// 区域名 → 下标；不认识返回 -1（调用方决定报错还是告警）。
int region_index(const std::vector<std::string>& regions, const std::string& name)
{
  for (size_t i = 0; i < regions.size(); ++i)
  {
    if (regions[i] == name) return static_cast<int>(i);
  }
  return -1;
}

// 读一个 [x, y] 二元组进 out；格式不对返回 false。
bool read_xy(const YAML::Node& node, Vec2& out)
{
  if (!node || node.size() != 2) return false;
  out.x = node[0].as<double>();
  out.y = node[1].as<double>();
  return true;
}
}  // namespace

bool load_config(const std::string& path, Config& cfg, std::string& error,
                 std::vector<std::string>& warnings)
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(path);
  }
  catch (const std::exception& e)
  {
    error = "打不开/解析失败 " + path + ": " + e.what();
    return false;
  }

  cfg = Config{};  // 从干净状态开始：调用方可能复用同一个对象（中途失败的加载会留下半截数据）
  error.clear();
  warnings.clear();

  // ---- 过洞总开关（显式，最先读：它决定 region/洞表整套要不要）----
  if (root["enable_tunnel"]) cfg.enable_tunnel = root["enable_tunnel"].as<bool>();

  // ---- 区域表：用户自定义的名字表，下标即区域号（区域判定是纯记账）----
  // region 与过洞耦合：enable_tunnel = false 时整套可省（不填 regions、点不标 region）。
  if (root["regions"])
  {
    for (const auto& it : root["regions"])
    {
      cfg.regions.push_back(it.as<std::string>());
    }
  }
  if (cfg.regions.empty())
  {
    if (cfg.enable_tunnel)
    {
      error = "regions 为空：过洞开着（enable_tunnel: true）就至少要定义一个区域"
              "（走廊可以就 [left, right]；不过洞请显式写 enable_tunnel: false）";
      return false;
    }
  }
  else
  {
    std::set<std::string> uniq;
    for (const auto& r : cfg.regions)
    {
      if (!uniq.insert(r).second)
      {
        error = "区域名重复: " + r;
        return false;
      }
    }
  }

  // ---- 开局区域（区域表存在才有意义；没有区域时保持默认 0，不参与任何判定）----
  if (!cfg.regions.empty())
  {
    std::string start_name = cfg.regions[0];
    read_str(root, "start_region", start_name);
    cfg.start_region = region_index(cfg.regions, start_name);
    if (cfg.start_region < 0)
    {
      error = "start_region 不在 regions 表里: " + start_name;
      return false;
    }
  }

  // ---- 地图对齐：场地系 → 导航建图系（align_map.py 解算后写进来）----
  if (root["field_to_map"])
  {
    const auto& n = root["field_to_map"];
    cfg.field_to_map.enabled = n["enabled"] ? n["enabled"].as<bool>() : true;
    read_double(n, "dx", cfg.field_to_map.dx);
    read_double(n, "dy", cfg.field_to_map.dy);
    read_double(n, "dyaw_deg", cfg.field_to_map.dyaw_deg);
  }

  // ---- 阈值（全部可选，缺省用代码里的默认值）----
  if (root["safety"])
  {
    const auto& s = root["safety"];
    read_double(s, "hp_home_ratio", cfg.hp_home_ratio);
    read_double(s, "hp_resume_ratio", cfg.hp_resume_ratio);
    read_int(s, "ammo_home_min", cfg.ammo_home_min);
    read_int(s, "ammo_resume_min", cfg.ammo_resume_min);
    read_double(s, "spin_w", cfg.spin_w);
    read_double(s, "odom_stale_ms", cfg.odom_stale_ms);
    read_double(s, "patrol_dwell_s", cfg.patrol_dwell_s);
    read_double(s, "switch_dwell_s", cfg.switch_dwell_s);
    read_int(s, "switch_confirm_frames", cfg.switch_confirm_frames);
    read_double(s, "gate_clear_m", cfg.gate_clear_m);
    read_double(s, "nav_timeout_s", cfg.nav_timeout_s);
    read_double(s, "nav_retry_s", cfg.nav_retry_s);
    read_double(s, "fold_timeout_s", cfg.fold_timeout_s);
    read_double(s, "tunnel_retry_cooldown_s", cfg.tunnel_retry_cooldown_s);
    read_double(s, "in_tunnel_margin_m", cfg.in_tunnel_margin_m);
    read_double(s, "arrive_radius_m", cfg.arrive_radius_m);
  }
  read_int(root, "mode_independent", cfg.mode_independent);

  // ---- 自动购买（决策层自己下单：阈值 + 金币门槛 + 冷却，详见 types.hpp 注释）----
  if (root["auto_buy"])
  {
    const auto& b = root["auto_buy"];
    if (b["enable"]) cfg.enable_auto_buy = b["enable"].as<bool>();
    read_int(b, "ammo_min_shots", cfg.ammo_buy_min_shots);
    read_int(b, "ammo_rounds", cfg.ammo_buy_rounds);
    read_int(b, "ammo_min_coins", cfg.ammo_buy_min_coins);
    read_double(b, "hp_ratio", cfg.hp_buy_ratio);
    read_int(b, "hp_amount", cfg.hp_buy_amount);
    read_int(b, "hp_min_coins", cfg.hp_buy_min_coins);
    read_int(b, "revive_min_coins", cfg.revive_buy_min_coins);
    read_double(b, "cooldown_s", cfg.buy_cooldown_s);
  }

  // ---- 点表：id / x / y / zone / region（region 仅过洞开着时必填），全部手填 ----
  if (root["points"])
  {
    for (const auto& it : root["points"])
    {
      PointDef p;
      p.id = it["id"].as<int>();
      p.pos.x = it["x"].as<double>();
      p.pos.y = it["y"].as<double>();
      const std::string zone = it["zone"].as<std::string>("patrol");
      p.zone = zone_from_str(zone);
      if (p.zone < 0)
      {
        error = "点 " + std::to_string(p.id) + " 的 zone 不认识: " + zone +
                "（只有 home/patrol）";
        return false;
      }
      const std::string rname = it["region"].as<std::string>("");
      p.region = region_index(cfg.regions, rname);
      if (p.region < 0)
      {
        if (cfg.enable_tunnel)
        {
          error = "点 " + std::to_string(p.id) + " 的 region 不认识: " + rname +
                  "（过洞开着时每个点都要标区域；不过洞请写 enable_tunnel: false）";
          return false;
        }
        p.region = -1;  // 不过洞：区域字段可省
      }
      cfg.points.push_back(p);
    }
  }

  // ---- 洞表：name + align（两个对正点，各带 region 绑定）+ 可选 center ----
  // enable_tunnel = false 时整段跳过（region 与过洞耦合，不过洞就不需要洞表）。
  if (root["tunnels"])
  {
    if (!cfg.enable_tunnel)
    {
      warnings.push_back("enable_tunnel = false：洞表已忽略（要过洞改回 true）");
    }
    else
    {
      for (const auto& it : root["tunnels"])
      {
        TunnelDef t;
        t.name = it["name"].as<std::string>("tunnel");
        const auto align = it["align"];
        if (!align || align.size() != 2)
        {
          error = "洞 " + t.name + " 的 align 必须正好两个对正点";
          return false;
        }
        for (int k = 0; k < 2; ++k)
        {
          if (!align[k]["p"] || !read_xy(align[k]["p"], t.align[k]))
          {
            error = "洞 " + t.name + " 的 align[" + std::to_string(k) +
                    "] 必须是 {p: [x, y], region: ...}";
            return false;
          }
          const std::string rname = align[k]["region"].as<std::string>("");
          t.align_region[k] = region_index(cfg.regions, rname);
          if (t.align_region[k] < 0)
          {
            error = "洞 " + t.name + " 的 align[" + std::to_string(k) +
                    "] 的 region 不认识: " + rname;
            return false;
          }
        }
        if (t.align_region[0] == t.align_region[1])
        {
          warnings.push_back("洞 " + t.name + " 两个对正点绑了同一个区域，这条洞不会参与路由");
          t.align_region[0] = t.align_region[1] = -1;
        }
        if (it["center"]) read_xy(it["center"], t.center);
        cfg.tunnels.push_back(t);
      }
    }
  }

  // ---- 话题表：sim / real 两套 ----
  if (root["topics"])
  {
    read_topics(root["topics"]["sim"], cfg.topics_sim);
    read_topics(root["topics"]["real"], cfg.topics_real);
  }

  // ---- 校验与告警（不致命，但要让人看见）----
  std::set<int> ids;
  for (const auto& p : cfg.points)
  {
    if (!ids.insert(p.id).second)
    {
      error = "点 id 重复: " + std::to_string(p.id);
      return false;
    }
  }
  if (cfg.points.empty())
  {
    error = "点表为空：至少要配一个家或巡逻点";
    return false;
  }
  {
    const auto has_zone = [&cfg](int zone) {
      for (const auto& p : cfg.points)
      {
        if (p.zone == zone) return true;
      }
      return false;
    };
    if (!has_zone(kZoneHome)) warnings.push_back("没配 home 点：血量低时无处可回（决策会落到停车）");
    if (!has_zone(kZonePatrol)) warnings.push_back("没配 patrol 点：平时没有巡逻目标（决策会退回家站着）");
  }
  // 孤岛区域 / 贴闸线：只在过洞开着时才有意义（region 与闸线都是过洞的概念）
  if (cfg.enable_tunnel)
  {
    // 孤岛区域：除开局区域外，没有任何洞连接的区域——属于它的点永远选不到
    for (int r = 0; r < static_cast<int>(cfg.regions.size()); ++r)
    {
      if (r == cfg.start_region) continue;
      bool linked = false;
      for (const auto& t : cfg.tunnels)
      {
        if (t.align_region[0] == r || t.align_region[1] == r) linked = true;
      }
      if (!linked)
        warnings.push_back("区域 " + cfg.regions[r] + " 没有任何洞连接（孤岛），属于它的点选不到");
    }
    for (const auto& t : cfg.tunnels)
    {
      if (std::hypot(t.align[0].x - t.align[1].x, t.align[0].y - t.align[1].y) < 1e-6)
        warnings.push_back("洞 " + t.name + " 的两个对正点重合，这条洞会被忽略");
    }
    for (const auto& p : cfg.points)
    {
      for (const auto& t : cfg.tunnels)
      {
        const double dx = t.align[1].x - t.align[0].x;
        const double dy = t.align[1].y - t.align[0].y;
        const double len2 = dx * dx + dy * dy;
        double u = len2 > 1e-12
                       ? ((p.pos.x - t.align[0].x) * dx + (p.pos.y - t.align[0].y) * dy) / len2
                       : 0.0;
        u = u < 0.0 ? 0.0 : (u > 1.0 ? 1.0 : u);
        const double d = std::hypot(p.pos.x - (t.align[0].x + u * dx),
                                    p.pos.y - (t.align[0].y + u * dy));
        if (d < cfg.gate_clear_m)
          warnings.push_back("点 " + std::to_string(p.id) + " 离洞 " + t.name +
                             " 的闸线太近（< gate_clear_m），选点时会被跳过");
      }
    }
  }

  // ---- 地图对齐生效：把全部点从场地系搬到导航建图系（一次性，之后全场只有地图系）----
  if (cfg.field_to_map.enabled)
  {
    const FrameTransform& ft = cfg.field_to_map;
    for (auto& p : cfg.points) p.pos = apply_frame_transform(p.pos, ft);
    for (auto& t : cfg.tunnels)
    {
      t.align[0] = apply_frame_transform(t.align[0], ft);
      t.align[1] = apply_frame_transform(t.align[1], ft);
      t.center = apply_frame_transform(t.center, ft);
    }
  }
  return true;
}

}  // namespace bbt_decide
