#pragma once

#include <string>
#include <vector>
#include "bbt_decide/types.hpp"

namespace bbt_decide
{
// All geometry is in the deployed map frame. Shared boundaries between different
// regions are deliberately unknown (-1); overlapping cells of one region are OK.
int region_for_position(const Config& cfg, const Vec2& position);

// Ordinary navigation stays in the selected region's closed union, including
// shared cell boundaries. Inspect analytic edge intersections, never sampling
// the path at a fixed step that could miss a narrow foreign/uncovered strip.
bool segment_within_region(const Config& cfg, const Vec2& start, const Vec2& end, int region);

// Open interior only: a configured entry/exit may lie on the tunnel boundary.
bool point_inside_tunnel(const Config& cfg, const Vec2& position);
bool segment_crosses_tunnel(const std::vector<TunnelDef>& tunnels,
                           const Vec2& start, const Vec2& end);
bool segment_crosses_tunnel(const Config& cfg, const Vec2& start, const Vec2& end);

// A successful continuous traversal may stop just inside its configured exit.
// Allow only a forward clearance through that exit, never a side exit/re-entry.
// positions starts with actual localization and includes every planned sample.
bool path_clears_tunnel_exit(const TunnelDef& tunnel, int exit_side,
                            const std::vector<Vec2>& positions);

// Validate a configured partition and the explicit point/entry region references.
// Empty geometry keeps legacy ID-only configurations usable; arbitrary positions
// in such configurations cannot be assigned to a region.
bool validate_geometry(const Config& cfg, std::string& error);
}  // namespace bbt_decide
