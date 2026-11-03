#pragma once

#include <string>
#include <vector>
#include "bbt_decide/types.hpp"

namespace bbt_decide
{
// All geometry is in the deployed map frame. Shared boundaries between different
// regions are deliberately unknown (-1); overlapping cells of one region are OK.
int region_for_position(const Config& cfg, const Vec2& position);

// Open interior only: a configured entry/exit may lie on the tunnel boundary.
bool point_inside_tunnel(const Config& cfg, const Vec2& position);

// Validate a configured partition and the explicit point/entry region references.
// Empty geometry keeps legacy ID-only configurations usable; arbitrary positions
// in such configurations cannot be assigned to a region.
bool validate_geometry(const Config& cfg, std::string& error);
}  // namespace bbt_decide
