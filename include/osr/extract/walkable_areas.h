#pragma once

#include <cstdint>
#include <array>
#include <filesystem>
#include <functional>
#include <memory>
#include <span>
#include <vector>

#include "geo/latlng.h"

#include "osr/types.h"

namespace osmium {
class Area;
class TagList;
class Way;
}  // namespace osmium

namespace osr {

struct ways;
struct way_properties;

// Walkable areas - plazas, station concourses, platforms mapped as polygons -
// as found during extraction, for area routing: pedestrians cross them in any
// direction, not only along the ways mapped onto them.
//
// osr does not route across areas itself. The extract hands every walkable
// area to a hook (see walkable_areas_hook) together with the nodes where the
// street network meets it. Implementations build their own graph inside the
// area from that - virtual nodes, joined to each other and to the connectors
// by virtual ways - and add it to the routing graph (see virtual_ways) before
// the routing graph is built from the ways.

// Which levels something is on, and its OSM layer.
//
// An object without a level tag is on every level, as foot routing treats
// kNoLevel: most of OSM is untagged, and reading that as "ground floor only"
// would disconnect far more than it fixes.
struct area_levels {
  static area_levels of(osmium::TagList const&);

  // For objects that share a node: the levels overlap.
  bool matches(area_levels const&) const;

  // For objects joined only by geometry - a way ending inside an area without
  // sharing a node with it: also the same layer. A footbridge over a plaza
  // lies inside its polygon and connects to nothing, and `layer` says so.
  bool matches_spatially(area_levels const&) const;

  bool any_{true};
  level_bits_t bits_{0U};
  int layer_{0};
};

// Is this area - a closed way (`from_way`) or a multipolygon - ground that
// pedestrians can cross in any direction?
bool is_walkable_area(osmium::TagList const&, bool from_way);

// Something inside an area that walking cannot pass.
struct blocker {
  enum class kind : std::uint8_t { kNone, kBuilding, kBarrier };

  static blocker of(osmium::TagList const&);

  // Does it block an area on `area`'s levels?
  bool stands_in(area_levels const& area) const;

  kind kind_{kind::kNone};
  area_levels levels_{};

  // Buildings: the lowest level the building occupies. Above 0 it stands on
  // columns - an upper floor over the plaza - and the ground underneath is
  // open.
  float min_level_{0.F};
};

struct area_ring {
  std::vector<osm_node_idx_t> osm_nodes_;
  std::vector<geo::latlng> points_;  // closed: front() == back()
};

// A node where the street network meets an area. It lies on a routing way,
// and becomes a routing node as soon as a virtual way uses it.
struct area_connector {
  osm_node_idx_t node_{};
  geo::latlng pos_{};

  // true: a node of the area's rings that a way passes through or ends at.
  // false: a way ends inside the area (stairs, a footway stub, ...).
  bool on_ring_{false};
};

struct walkable_area {
  std::int64_t osm_id_{0};  // of the way or the multipolygon relation
  bool from_way_{false};
  area_levels levels_{};

  // rings_[0] is the outline, the others are holes. A multipolygon with
  // several outer rings yields one walkable_area per outer ring.
  std::vector<area_ring> rings_;

  // Inside the area, on its levels. Buildings are closed rings, barriers
  // single segments. They are not cleaned: a building can overlap a hole
  // (often it _is_ the hole) or reach past the outline.
  std::vector<std::vector<geo::latlng>> buildings_;
  std::vector<std::array<geo::latlng, 2U>> barriers_;

  // Sorted by OSM node id.
  std::vector<area_connector> connectors_;
};

// Ways the walkable areas hook adds to the routing graph. They are added
// before ways::connect_ways(), so they become ordinary ways: every node they
// share with another way is a routing node, and everything built afterwards -
// components, the lookup rtree, contraction - sees them like any other way.
//
// Their OSM ids (nodes and ways) follow right after the highest id in the
// input file - not only the highest routing one: the nodes pass looks nodes
// up by id, turn restrictions look up ways by id.
//
// The cost of a virtual edge, as of any edge, follows from the length of its
// polyline and its way_properties. At most kMaxWaysPerNode ways can meet at a
// node (connect_ways() throws otherwise); a way passing through a node
// counts once.
struct virtual_ways {
  // Open ground on `level`, for walking only.
  static way_properties foot_way(level_t level);

  // `max_node_id` / `max_way_id`: the highest ids in the input file.
  virtual_ways(ways&, std::int64_t max_node_id, std::int64_t max_way_id);

  bool is_virtual(osm_node_idx_t const n) const {
    return to_idx(n) >= first_node_id_ &&
           to_idx(n) < first_node_id_ + nodes_.size();
  }
  bool is_virtual(osm_way_idx_t const w) const {
    return to_idx(w) >= first_way_id_ && to_idx(w) < next_way_id_;
  }

  // A virtual node. If it becomes a routing node (two ways share it), it is
  // accessible on foot, by bike and by car - the ways decide - on `level`.
  osm_node_idx_t add_node(geo::latlng const&, level_t level = kNoLevel);

  // A way through `nodes` - virtual nodes and real ones, e.g. connectors -
  // with one point of `polyline` per node. Not oneway unless `p` says so.
  way_idx_t add_way(way_properties const& p,
                    std::span<osm_node_idx_t const> nodes,
                    std::span<geo::latlng const> polyline);

  // After the nodes pass: positions and properties of the virtual nodes that
  // became routing nodes.
  void set_node_properties();

  struct node {
    geo::latlng pos_;
    level_t level_;
  };

  ways& w_;
  std::uint64_t first_node_id_;
  std::uint64_t first_way_id_;
  std::uint64_t next_way_id_;
  std::vector<node> nodes_;  // OSM id first_node_id_ + i
};

// Called once during extract(), after all ways were read and before the
// routing graph is built from them, with every walkable area in the data. The
// ways are not connected yet: their polylines, OSM nodes and properties are
// there, routing nodes are not. `out` is the extract's output directory, so
// the hook can store what it builds next to osr's own files.
using walkable_areas_hook =
    std::function<void(std::filesystem::path const& out,
                       ways const&,
                       std::vector<walkable_area> const&,
                       virtual_ways&)>;

// Collects walkable areas during extract(); not needed outside of it.
struct walkable_area_collector {
  explicit walkable_area_collector(ways&);
  ~walkable_area_collector();

  walkable_area_collector(walkable_area_collector const&) = delete;
  walkable_area_collector& operator=(walkable_area_collector const&) = delete;

  // Pass over all ways, each right after way_handler saw it (serially, in
  // order): closed walkable ways, and the levels of every routing way.
  void way(osmium::Way const&);

  // Assembled multipolygon relations (osm::build_area: the area's id is the
  // relation's).
  void multipolygon(osmium::Area const&);

  // Once all areas are in: finds the connectors.
  void find_connectors();

  // Buildings and barriers, after find_connectors().
  void blocker_way(osmium::Way const&);

  // After blocker_way().
  std::vector<walkable_area> finish();

  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr
