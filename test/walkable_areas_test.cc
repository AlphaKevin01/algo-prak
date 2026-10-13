#include "gtest/gtest.h"

#include <algorithm>
#include <array>
#include <filesystem>
#include <vector>

#include "osr/extract/extract.h"
#include "osr/extract/walkable_areas.h"
#include "osr/lookup.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/ways.h"
#include "xml_to_pbf.h"

namespace fs = std::filesystem;
using namespace osr;

namespace {

// lat = 49 + y / 10000, lon = 8 + x / 10000 (osm ids sorted, as the
// extract requires)
//
//  A: way 100, highway=pedestrian + area=yes, (0,0)-(10,10).
//     Footway 402 starts at its corner 3.
//  B: multipolygon 300 (outline way 200 untagged), (20,0)-(30,10), with a
//     hole (24,4)-(26,6). Footway 401 crosses its outline at ring node 19
//     and ends inside at 33. A footbridge (layer=1) ends inside at 43. A
//     building stands inside, a fence crosses the outline.
constexpr auto const kAreasOsm = R"osm(<osm version="0.6" generator="test">
  <node id="1" lat="49.0000" lon="8.0000"/>
  <node id="2" lat="49.0010" lon="8.0000"/>
  <node id="3" lat="49.0010" lon="8.0010"/>
  <node id="4" lat="49.0000" lon="8.0010"/>
  <node id="11" lat="49.0000" lon="8.0020"/>
  <node id="12" lat="49.0010" lon="8.0020"/>
  <node id="13" lat="49.0010" lon="8.0030"/>
  <node id="14" lat="49.0000" lon="8.0030"/>
  <node id="15" lat="49.0004" lon="8.0024"/>
  <node id="16" lat="49.0006" lon="8.0024"/>
  <node id="17" lat="49.0006" lon="8.0026"/>
  <node id="18" lat="49.0004" lon="8.0026"/>
  <node id="19" lat="49.0005" lon="8.0020"/>
  <node id="32" lat="49.0005" lon="8.0015"/>
  <node id="33" lat="49.0005" lon="8.0022"/>
  <node id="41" lat="49.0015" lon="8.0015"/>
  <node id="42" lat="49.0012" lon="8.0028"/>
  <node id="43" lat="49.0008" lon="8.0028"/>
  <node id="51" lat="49.0001" lon="8.0021"/>
  <node id="52" lat="49.0002" lon="8.0021"/>
  <node id="53" lat="49.0002" lon="8.0022"/>
  <node id="54" lat="49.0001" lon="8.0022"/>
  <node id="61" lat="49.0005" lon="8.0029"/>
  <node id="62" lat="49.0005" lon="8.0031"/>
  <way id="100">
    <nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/>
    <tag k="highway" v="pedestrian"/>
    <tag k="area" v="yes"/>
  </way>
  <way id="200">
    <nd ref="11"/><nd ref="19"/><nd ref="12"/><nd ref="13"/><nd ref="14"/>
    <nd ref="11"/>
  </way>
  <way id="201">
    <nd ref="15"/><nd ref="16"/><nd ref="17"/><nd ref="18"/><nd ref="15"/>
  </way>
  <way id="401">
    <nd ref="32"/><nd ref="19"/><nd ref="33"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="402">
    <nd ref="3"/><nd ref="41"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="403">
    <nd ref="42"/><nd ref="43"/>
    <tag k="highway" v="footway"/>
    <tag k="bridge" v="yes"/>
    <tag k="layer" v="1"/>
  </way>
  <way id="500">
    <nd ref="51"/><nd ref="52"/><nd ref="53"/><nd ref="54"/><nd ref="51"/>
    <tag k="building" v="yes"/>
  </way>
  <way id="501">
    <nd ref="61"/><nd ref="62"/>
    <tag k="barrier" v="fence"/>
  </way>
  <relation id="300">
    <member type="way" ref="200" role="outer"/>
    <member type="way" ref="201" role="inner"/>
    <tag k="type" v="multipolygon"/>
    <tag k="highway" v="pedestrian"/>
  </relation>
</osm>)osm";

}  // namespace

TEST(walkable_areas, extract_hook) {
  auto const dir = fs::temp_directory_path() / "osr_walkable_areas_test";
  auto const osm = osr::test::write_osm_pbf("osr_walkable_areas", kAreasOsm);

  // A hub inside B, right of the hole, joined to the connectors 19 and 33.
  auto const hub_pos = geo::latlng{49.0002, 8.0028};
  auto hub = osm_node_idx_t{};
  auto areas = std::vector<walkable_area>{};
  extract(false, osm, dir, {},
          [&](fs::path const& out, ways const&,
              std::vector<walkable_area> const& found, virtual_ways& vw) {
            EXPECT_EQ(dir, out);
            areas = found;
            auto const b =
                std::ranges::find(found, 300, &walkable_area::osm_id_);
            ASSERT_NE(end(found), b);
            ASSERT_EQ(2U, b->connectors_.size());
            hub = vw.add_node(hub_pos);
            EXPECT_TRUE(vw.is_virtual(hub));
            EXPECT_FALSE(vw.is_virtual(to_osm_node_idx(62)));  // highest id
            for (auto const& c : b->connectors_) {
              auto const nodes = std::array{c.node_, hub};
              auto const polyline = std::array{c.pos_, hub_pos};
              vw.add_way(virtual_ways::foot_way(kNoLevel), nodes, polyline);
            }
          });

  ASSERT_EQ(2U, areas.size());
  std::ranges::sort(areas, {}, &walkable_area::osm_id_);
  auto const& a = areas[0];
  auto const& b = areas[1];

  EXPECT_EQ(100, a.osm_id_);
  EXPECT_TRUE(a.from_way_);
  EXPECT_EQ(1U, a.rings_.size());
  EXPECT_EQ(5U, a.rings_[0].points_.size());
  ASSERT_EQ(1U, a.connectors_.size());
  EXPECT_EQ(to_osm_node_idx(3), a.connectors_[0].node_);
  EXPECT_TRUE(a.connectors_[0].on_ring_);
  EXPECT_TRUE(a.buildings_.empty());
  EXPECT_TRUE(a.barriers_.empty());

  EXPECT_EQ(300, b.osm_id_);
  EXPECT_FALSE(b.from_way_);
  EXPECT_EQ(2U, b.rings_.size());  // outline + hole

  // 19 on the outline, 33 the end of footway 401 inside. The footbridge end
  // (43) is on another layer.
  ASSERT_EQ(2U, b.connectors_.size());
  EXPECT_EQ(to_osm_node_idx(19), b.connectors_[0].node_);
  EXPECT_TRUE(b.connectors_[0].on_ring_);
  EXPECT_EQ(to_osm_node_idx(33), b.connectors_[1].node_);
  EXPECT_FALSE(b.connectors_[1].on_ring_);

  ASSERT_EQ(1U, b.buildings_.size());
  EXPECT_EQ(5U, b.buildings_[0].size());
  EXPECT_EQ(1U, b.barriers_.size());

  // The virtual ways are part of the graph: the hub and 33 (otherwise the end
  // of a single way) are routing nodes, and a walk reaches the hub across B.
  auto const w = ways{dir, cista::mmap::protection::READ};
  auto const l = lookup{w, dir, cista::mmap::protection::READ};
  auto const hub_node = w.find_node_idx(hub);
  ASSERT_TRUE(hub_node.has_value());
  EXPECT_TRUE(w.r_->node_properties_[*hub_node].is_walk_accessible());
  EXPECT_LT(geo::distance(w.get_node_pos(*hub_node).as_latlng(), hub_pos), 1.0);
  EXPECT_TRUE(w.find_node_idx(to_osm_node_idx(33)).has_value());

  auto const p = route(
      foot<false, elevator_tracking>::parameters{}, w, l, search_profile::kFoot,
      location{{49.0005, 8.0015}, kNoLevel}, location{hub_pos, kNoLevel},
      std::chrono::seconds{3600}, direction::kForward, 10.0);
  ASSERT_TRUE(p.has_value());
  // 36m along footway 401 to 19, then 67m along the virtual way to the hub.
  // No real way is within the match distance of the hub.
  EXPECT_NEAR(103.0, p->dist_, 5.0);
}

TEST(walkable_areas, no_hook_no_change) {
  auto const dir = fs::temp_directory_path() / "osr_walkable_areas_test_plain";
  auto const osm = osr::test::write_osm_pbf("osr_walkable_areas", kAreasOsm);

  extract(false, osm, dir, {});

  auto const w = ways{dir, cista::mmap::protection::READ};
  EXPECT_FALSE(w.find_node_idx(to_osm_node_idx(33)).has_value());
}
