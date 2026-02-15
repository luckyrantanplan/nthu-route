#include <gtest/gtest.h>

#include "../src/grdb/RoutingRegion.h"

namespace {

TEST(RoutingRegionTest, Construction) {
    NTHUR::RoutingRegion rr(3, 4, 2);
    ASSERT_EQ(rr.get_gridx(), 3);
    ASSERT_EQ(rr.get_gridy(), 4);
    ASSERT_EQ(rr.get_layerNumber(), 2);
    ASSERT_EQ(rr.get_netNumber(), 0u);
}

TEST(RoutingRegionTest, TileTransformDefaults) {
    NTHUR::RoutingRegion rr(3, 3, 1);
    ASSERT_EQ(rr.get_llx(), 0);
    ASSERT_EQ(rr.get_lly(), 0);
    ASSERT_EQ(rr.get_tileWidth(), 1);
    ASSERT_EQ(rr.get_tileHeight(), 1);
}

TEST(RoutingRegionTest, SetTileTransform) {
    NTHUR::RoutingRegion rr(3, 3, 1);
    rr.setTileTransformInformation(10, 20, 5, 6);
    ASSERT_EQ(rr.get_llx(), 10);
    ASSERT_EQ(rr.get_lly(), 20);
    ASSERT_EQ(rr.get_tileWidth(), 5);
    ASSERT_EQ(rr.get_tileHeight(), 6);
}

TEST(RoutingRegionTest, AddNetWithMultiplePins) {
    NTHUR::RoutingRegion rr(10, 10, 2);
    rr.setNetNumber(1);
    rr.beginAddANet("netA", 0, 3, 1);
    rr.addPin(0, 0, 0);
    rr.addPin(5, 0, 0);
    rr.addPin(0, 5, 0);
    rr.endAddANet();
    ASSERT_EQ(rr.get_netNumber(), 1u);
    ASSERT_EQ(rr.get_net(0).get_name(), "netA");
    ASSERT_EQ(rr.get_net(0).get_pinList().size(), 3u);
}

TEST(RoutingRegionTest, SinglePinNetRemoved) {
    NTHUR::RoutingRegion rr(10, 10, 1);
    rr.setNetNumber(1);
    rr.beginAddANet("singlePin", 0, 1, 0);
    rr.addPin(0, 0, 0);
    rr.endAddANet();
    // Nets with <= 1 pin are removed
    ASSERT_EQ(rr.get_netNumber(), 0u);
}

TEST(RoutingRegionTest, DuplicatePinsDeduplicated) {
    NTHUR::RoutingRegion rr(10, 10, 1);
    rr.setNetNumber(1);
    rr.beginAddANet("dupNet", 0, 3, 0);
    rr.addPin(5, 5, 0);
    rr.addPin(5, 5, 0);  // duplicate tile
    rr.addPin(3, 3, 0);
    rr.endAddANet();
    ASSERT_EQ(rr.get_netNumber(), 1u);
    // Duplicate pins at same tile should be deduplicated
    ASSERT_EQ(rr.get_net(0).get_pinList().size(), 2u);
}

TEST(RoutingRegionTest, SetCapacities) {
    NTHUR::RoutingRegion rr(3, 3, 2);
    // These should not throw
    rr.setVerticalCapacity(0, 5);
    rr.setVerticalCapacity(1, 3);
    rr.setHorizontalCapacity(0, 4);
    rr.setHorizontalCapacity(1, 6);
}

TEST(RoutingRegionTest, AdjustEdgeCapacity) {
    NTHUR::RoutingRegion rr(3, 3, 2);
    rr.setVerticalCapacity(0, 5);
    rr.setHorizontalCapacity(0, 5);
    // Adjust a specific edge
    rr.adjustEdgeCapacity(0, 0, 0, 1, 0, 0, 2);
    // Verify the adjusted capacity through the max capacity plane
    const auto& cap = rr.getMaxCapacity();
    ASSERT_EQ(cap.edge(NTHUR::Coordinate_3d(0, 0, 0), NTHUR::Coordinate_3d(1, 0, 0)), 2);
}

}  // namespace
