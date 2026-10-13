#include "osr/extract/walkable_areas.h"

#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <array>
#include <initializer_list>
#include <mutex>
#include <optional>
#include <ranges>
#include <string_view>
#include <utility>

#include "osmium/osm/area.hpp"
#include "osmium/osm/way.hpp"

#include "geo/box.h"

#include "utl/enumerate.h"
#include "utl/parallel_for.h"
#include "utl/verify.h"
#include "utl/zip.h"

#include "rtree.h"
#include "tg.h"

#include "osr/util/levels.h"
#include "osr/ways.h"

namespace osr {

namespace {

bool one_of(char const* value,
            std::initializer_list<std::string_view> const options) {
  return value != nullptr &&
         std::ranges::find(options, std::string_view{value}) != end(options);
}

std::optional<float> number(char const* value) {
  if (value == nullptr) {
    return std::nullopt;
  }
  char* end = nullptr;
  auto const x = std::strtof(value, &end);
  return end == value ? std::nullopt : std::optional{x};
}

float highest_level(level_bits_t const bits) {
  auto highest = kMinLevel;
  for (auto b = 0U; b != 64U; ++b) {
    if (((bits >> b) & 1U) != 0U) {
      highest =
          std::max(highest, level_t{static_cast<std::uint8_t>(b)}.to_float());
    }
  }
  return highest;
}

geo::latlng to_latlng(osmium::Location const& l) { return {l.lat(), l.lon()}; }

template <typename NodeRefs>
std::optional<area_ring> to_ring(NodeRefs const& nodes) {
  auto r = area_ring{};
  for (auto const& n : nodes) {
    if (!n.location().valid()) {
      return std::nullopt;
    }
    r.osm_nodes_.push_back(to_osm_node_idx(n.ref()));
    r.points_.push_back(to_latlng(n.location()));
  }
  return r;
}

}  // namespace

area_levels area_levels::of(osmium::TagList const& tags) {
  auto l = area_levels{};
  for (auto const* key : {"level", "indoor:level"}) {
    if (auto const* v = tags[key]; v != nullptr) {
      l.any_ = false;
      l.bits_ |= parse_levels(v);
    }
  }
  if (auto const* layer = tags["layer"]; layer != nullptr) {
    l.layer_ = std::atoi(layer);
  }
  return l;
}

bool area_levels::matches(area_levels const& o) const {
  return any_ || o.any_ || (bits_ & o.bits_) != 0U;
}

bool area_levels::matches_spatially(area_levels const& o) const {
  return layer_ == o.layer_ && matches(o);
}

bool is_walkable_area(osmium::TagList const& tags, bool const from_way) {
  // A station covers a whole station on every level; walking across it
  // would teleport between them.
  if (one_of(tags["public_transport"], {"station"}) ||
      one_of(tags["area"], {"no"})) {
    return false;
  }

  auto const is_surface =
      one_of(tags["highway"], {"pedestrian", "footway", "path", "living_street",
                               "service", "corridor", "platform", "steps"}) ||
      one_of(tags["public_transport"], {"platform"}) ||
      one_of(tags["railway"], {"platform"});
  if (!is_surface) {
    return false;
  }

  // A closed highway=footway without area=yes is a loop, not a surface. A
  // multipolygon is an area by construction.
  return !from_way || one_of(tags["area"], {"yes"});
}

blocker blocker::of(osmium::TagList const& tags) {
  auto b = blocker{.levels_ = area_levels::of(tags)};
  if (one_of(tags["barrier"], {"yes", "wall", "fence", "hedge",
                               "retaining_wall", "city_wall", "guard_rail"})) {
    b.kind_ = kind::kBarrier;
    return b;
  }

  // building=roof is a roof on posts - a petrol station, a platform canopy -
  // with open ground underneath.
  auto const* building = tags["building"];
  if (building == nullptr || one_of(building, {"no", "roof"})) {
    return b;
  }
  b.kind_ = kind::kBuilding;
  if (auto const l = number(tags["building:min_level"]); l.has_value()) {
    b.min_level_ = *l;
  } else if (auto const h = number(tags["min_height"]);
             h.has_value() && *h > 2.5F) {
    b.min_level_ = 1.F;  // lifted by more than a storey: an upper floor
  }
  return b;
}

bool blocker::stands_in(area_levels const& area) const {
  switch (kind_) {
    case kind::kNone: return false;

    // A fence on a bridge does not block the plaza underneath, so a barrier
    // has to be on the area's layer.
    case kind::kBarrier: return area.matches_spatially(levels_);

    // A building's `layer` orders it against the tunnels and bridges around
    // it, it does not lift it off the ground. Its level has to match, and an
    // upper floor on columns (building:min_level=1) only blocks the levels it
    // is on.
    case kind::kBuilding:
      if (min_level_ > 0.F &&
          (area.any_ || highest_level(area.bits_) < min_level_)) {
        return false;
      }
      return area.matches(levels_);
  }
  return false;
}

struct walkable_area_collector::impl {
  explicit impl(ways& w) : w_{w} {}

  ~impl() {
    for (auto* g : geoms_) {
      tg_geom_free(g);
    }
    if (rtree_ != nullptr) {
      rtree_free(rtree_);
    }
  }

  void build_index() {
    rtree_ = rtree_new();
    utl::verify(rtree_ != nullptr, "walkable area rtree creation failed");

    auto pts = std::vector<tg_point>{};
    auto const make_ring = [&](area_ring const& r) {
      pts.clear();
      for (auto const& p : r.points_) {
        pts.push_back(tg_point{p.lng(), p.lat()});
      }
      return tg_ring_new(pts.data(), static_cast<int>(pts.size()));
    };

    auto holes = std::vector<tg_ring*>{};
    for (auto const [i, a] : utl::enumerate(areas_)) {
      auto* outer = make_ring(a.rings_.front());
      holes.clear();
      for (auto r = 1U; r < a.rings_.size(); ++r) {
        holes.push_back(make_ring(a.rings_[r]));
      }
      auto* poly =
          tg_poly_new(outer, holes.data(), static_cast<int>(holes.size()));
      geoms_.push_back(tg_geom_new_polygon(poly));
      tg_poly_free(poly);
      tg_ring_free(outer);
      for (auto* h : holes) {
        tg_ring_free(h);
      }

      auto box = geo::box{};
      for (auto const& p : a.rings_.front().points_) {
        box.extend(p);
      }
      auto const min = box.min_.lnglat();
      auto const max = box.max_.lnglat();
      rtree_insert(rtree_, min.data(), max.data(), reinterpret_cast<void*>(i));

      auto const idx = static_cast<std::uint32_t>(i);
      for (auto const& r : a.rings_) {
        for (auto const n : r.osm_nodes_) {
          auto& areas = ring_areas_[n];
          if (areas.empty() || areas.back() != idx) {
            areas.push_back(idx);
          }
        }
      }
    }
  }

  // Calls fn(area) for every area `pos` lies in (or on the outline of).
  template <typename Fn>
  void for_each_area_at(geo::latlng const& pos, Fn&& fn) const {
    struct state {
      impl const& impl_;
      geo::latlng const& pos_;
      Fn& fn_;
    } s{*this, pos, fn};
    auto const xy = pos.lnglat();
    rtree_search(
        rtree_, xy.data(), xy.data(),
        [](double const*, double const*, void const* item, void* udata) {
          auto& st = *static_cast<state*>(udata);
          auto const a = reinterpret_cast<std::size_t>(item);
          if (tg_geom_intersects_xy(st.impl_.geoms_[a], st.pos_.lng(),
                                    st.pos_.lat())) {
            st.fn_(static_cast<std::uint32_t>(a));
          }
          return true;
        },
        &s);
  }

  bool is_ring_node(std::uint32_t const area, osm_node_idx_t const n) const {
    auto const it = ring_areas_.find(n);
    return it != end(ring_areas_) &&
           std::ranges::find(it->second, area) != end(it->second);
  }

  ways& w_;
  std::vector<area_levels> way_levels_;  // per way_idx_t
  std::vector<walkable_area> areas_;
  std::vector<std::vector<area_connector>> connectors_;  // per area

  hash_map<osm_node_idx_t, std::vector<std::uint32_t>> ring_areas_;
  std::vector<tg_geom*> geoms_;  // per area
  rtree* rtree_{nullptr};
};

walkable_area_collector::walkable_area_collector(ways& w)
    : impl_{std::make_unique<impl>(w)} {}

walkable_area_collector::~walkable_area_collector() = default;

void walkable_area_collector::way(osmium::Way const& w) {
  auto& i = *impl_;

  // way_handler has just appended this way if it is a routing way.
  if (i.w_.n_ways() > i.way_levels_.size() &&
      i.w_.way_osm_idx_[way_idx_t{i.w_.n_ways() - 1U}] ==
          to_osm_way_idx(w.id())) {
    i.way_levels_.push_back(area_levels::of(w.tags()));
  }

  auto const& nodes = w.nodes();
  if (nodes.size() < 4U || nodes.front().ref() != nodes.back().ref() ||
      !is_walkable_area(w.tags(), true)) {
    return;
  }
  if (auto ring = to_ring(nodes); ring.has_value()) {
    auto& a = i.areas_.emplace_back();
    a.osm_id_ = w.id();
    a.from_way_ = true;
    a.levels_ = area_levels::of(w.tags());
    a.rings_.push_back(std::move(*ring));
  }
}

void walkable_area_collector::multipolygon(osmium::Area const& mp) {
  if (!is_walkable_area(mp.tags(), false)) {
    return;
  }

  // As far as crossing goes, every outer ring is an area of its own.
  for (auto const& outer : mp.outer_rings()) {
    auto a = walkable_area{};
    a.osm_id_ = mp.id();
    a.levels_ = area_levels::of(mp.tags());
    auto r = to_ring(outer);
    if (!r.has_value() || r->points_.size() < 4U) {
      continue;
    }
    a.rings_.push_back(std::move(*r));
    for (auto const& inner : mp.inner_rings(outer)) {
      if (auto h = to_ring(inner); h.has_value() && h->points_.size() >= 4U) {
        a.rings_.push_back(std::move(*h));
      }
    }
    impl_->areas_.push_back(std::move(a));
  }
}

void walkable_area_collector::find_connectors() {
  auto& i = *impl_;
  utl::verify(i.way_levels_.size() == i.w_.n_ways(),
              "walkable areas: levels of {} ways, but {} ways",
              i.way_levels_.size(), i.w_.n_ways());

  i.build_index();
  i.connectors_.resize(i.areas_.size());

  auto mutex = std::mutex{};
  utl::parallel_for_run(i.w_.n_ways(), [&](std::size_t const x) {
    auto const way = way_idx_t{x};
    auto const nodes = i.w_.way_osm_nodes_[way];
    auto const polyline = i.w_.way_polylines_[way];
    auto const& levels = i.way_levels_[x];

    auto found = std::vector<std::pair<std::uint32_t, area_connector>>{};

    // Ring nodes the way passes through or ends at.
    for (auto const [n, p] : utl::zip(nodes, polyline)) {
      if (auto const it = i.ring_areas_.find(n); it != end(i.ring_areas_)) {
        for (auto const a : it->second) {
          if (i.areas_[a].levels_.matches(levels)) {
            found.emplace_back(
                a, area_connector{
                       .node_ = n, .pos_ = p.as_latlng(), .on_ring_ = true});
          }
        }
      }
    }

    // A way all of whose nodes are ring nodes of an area is (part of) its
    // outline, not something that meets it.
    auto outline_of = std::vector<std::uint32_t>{};
    for (auto const& [a, _] : found) {
      if (static_cast<std::size_t>(std::ranges::count(
              found, a, &std::pair<std::uint32_t, area_connector>::first)) ==
          nodes.size()) {
        outline_of.push_back(a);
      }
    }
    std::erase_if(found, [&](auto const& f) {
      return std::ranges::find(outline_of, f.first) != end(outline_of);
    });

    // Ends inside an area: stairs, lifts, footway stubs.
    for (auto const e : {std::size_t{0U}, nodes.size() - 1U}) {
      auto const n = nodes[e];
      auto const pos = polyline[e].as_latlng();
      i.for_each_area_at(pos, [&](std::uint32_t const a) {
        if (!i.is_ring_node(a, n) &&
            i.areas_[a].levels_.matches_spatially(levels)) {
          found.emplace_back(
              a, area_connector{.node_ = n, .pos_ = pos, .on_ring_ = false});
        }
      });
    }

    if (!found.empty()) {
      auto const lock = std::scoped_lock{mutex};
      for (auto const& [a, c] : found) {
        i.connectors_[a].push_back(c);
      }
    }
  });
}

void walkable_area_collector::blocker_way(osmium::Way const& w) {
  auto const b = blocker::of(w.tags());
  if (b.kind_ == blocker::kind::kNone) {
    return;
  }

  auto& i = *impl_;
  auto const r = to_ring(w.nodes());
  if (!r.has_value() || r->points_.size() < 2U) {
    return;
  }

  // The areas the blocker stands in: those with one of its nodes inside that
  // is not a ring node - a fence along the outline is the area's edge, and a
  // building mapped as a hole has all its nodes on the ring.
  auto const n = r->points_.size();
  auto inside = std::vector<std::vector<std::uint32_t>>(n);
  auto touched = std::vector<std::uint32_t>{};
  for (auto k = 0U; k != n; ++k) {
    i.for_each_area_at(r->points_[k], [&](std::uint32_t const a) {
      if (!i.is_ring_node(a, r->osm_nodes_[k]) &&
          b.stands_in(i.areas_[a].levels_)) {
        inside[k].push_back(a);
        touched.push_back(a);
      }
    });
  }
  std::ranges::sort(touched);
  touched.erase(std::ranges::unique(touched).begin(), end(touched));

  auto const closed = n >= 4U && r->osm_nodes_.front() == r->osm_nodes_.back();
  for (auto const a : touched) {
    auto& area = i.areas_[a];
    if (b.kind_ == blocker::kind::kBuilding) {
      if (closed) {
        area.buildings_.push_back(r->points_);
      }
      continue;
    }

    // Keep a segment if either end is inside: a fence crossing the outline
    // blocks just as much as one wholly within it.
    auto const in = [&](std::size_t const k) {
      return std::ranges::find(inside[k], a) != end(inside[k]);
    };
    for (auto k = 0U; k + 1U < n; ++k) {
      if (in(k) || in(k + 1U)) {
        area.barriers_.push_back({r->points_[k], r->points_[k + 1U]});
      }
    }
  }
}

std::vector<walkable_area> walkable_area_collector::finish() {
  auto& i = *impl_;
  for (auto const [a, cs] : utl::enumerate(i.connectors_)) {
    // A node can be found several times (closed ways, ways meeting), and as
    // ring node and way end at once: on the ring wins.
    std::ranges::sort(cs, [](area_connector const& x, area_connector const& y) {
      return std::pair{x.node_, !x.on_ring_} < std::pair{y.node_, !y.on_ring_};
    });
    auto const dups = std::ranges::unique(cs, {}, &area_connector::node_);
    cs.erase(dups.begin(), dups.end());
    i.areas_[a].connectors_ = std::move(cs);
  }
  return std::move(i.areas_);
}

way_properties virtual_ways::foot_way(level_t const level) {
  auto p = way_properties{};
  std::memset(&p, 0, sizeof(way_properties));
  p.is_foot_accessible_ = true;
  p.from_level_ = to_idx(level);
  p.to_level_ = to_idx(level);
  return p;
}

virtual_ways::virtual_ways(ways& w,
                           std::int64_t const max_node_id,
                           std::int64_t const max_way_id)
    : w_{w},
      first_node_id_{
          static_cast<std::uint64_t>(std::max(max_node_id, std::int64_t{0})) +
          1U},
      first_way_id_{
          static_cast<std::uint64_t>(std::max(max_way_id, std::int64_t{0})) +
          1U},
      next_way_id_{first_way_id_} {}

osm_node_idx_t virtual_ways::add_node(geo::latlng const& pos,
                                      level_t const level) {
  nodes_.push_back(node{.pos_ = pos, .level_ = level});
  return osm_node_idx_t{first_node_id_ + nodes_.size() - 1U};
}

way_idx_t virtual_ways::add_way(way_properties const& p,
                                std::span<osm_node_idx_t const> nodes,
                                std::span<geo::latlng const> polyline) {
  utl::verify(nodes.size() == polyline.size() && nodes.size() >= 2U,
              "virtual way: {} nodes, {} points", nodes.size(),
              polyline.size());
  for (auto const n : nodes) {
    utl::verify(to_idx(n) < first_node_id_ + nodes_.size() ||
                    is_negative_encoded_osm_id(to_idx(n)),
                "virtual way: unknown virtual node {}", n);
  }

  auto const way = way_idx_t{w_.n_ways()};
  w_.way_osm_idx_.push_back(osm_way_idx_t{next_way_id_++});
  w_.r_->way_properties_.emplace_back(p);
  w_.way_polylines_.emplace_back(polyline |
                                 std::views::transform(point::from_latlng));
  w_.way_osm_nodes_.emplace_back(nodes);
  w_.way_names_.emplace_back(string_idx_t::invalid());
  w_.way_has_conditional_access_no_.resize(to_idx(way) + 1U);
  for (auto const n : nodes) {
    w_.node_way_counter_.increment(to_idx(n));
  }
  return way;
}

void virtual_ways::set_node_properties() {
  for (auto const [i, n] : utl::enumerate(nodes_)) {
    auto const idx = w_.find_node_idx(osm_node_idx_t{first_node_id_ + i});
    if (!idx.has_value()) {
      continue;  // on a single way: geometry only
    }
    auto p = node_properties{};
    std::memset(&p, 0, sizeof(node_properties));
    p.is_foot_accessible_ = true;
    p.is_bike_accessible_ = true;
    p.is_car_accessible_ = true;
    p.is_bus_accessible_ = true;
    p.from_level_ = to_idx(n.level_);
    p.to_level_ = to_idx(n.level_);
    w_.r_->node_properties_[*idx] = p;
    w_.r_->node_positions_[*idx] = point::from_latlng(n.pos_);
  }
}

}  // namespace osr
