#pragma once

#include <filesystem>

#include "osr/extract/walkable_areas.h"

namespace osr {

// `on_walkable_areas`: optional, see walkable_areas_hook. Without it, walkable
// areas are not collected and the graph is the same as before.
void extract(bool with_platforms,
             std::filesystem::path const& in,
             std::filesystem::path const& out,
             std::filesystem::path const& elevation_dir,
             walkable_areas_hook const& on_walkable_areas = {});

}  // namespace osr
