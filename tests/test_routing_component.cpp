#include <gtest/gtest.h>

#include "../src/grdb/RoutingComponent.h"

namespace {

TEST(NetTest, Construction) {
    NTHUR::Net net("testNet", 42, 0, 1);
    ASSERT_EQ(net.get_name(), "testNet");
    ASSERT_EQ(net.serialNumber, 42);
    ASSERT_EQ(net.id, 0);
    ASSERT_EQ(net.minWireWidth, 1);
}

TEST(NetTest, EmptyBoundingBox) {
    NTHUR::Net net("empty", 0, 0, 0);
    ASSERT_EQ(net.get_bboxSize(), 0);
    ASSERT_TRUE(net.get_pinList().empty());
}

TEST(NetTest, AddPinsUpdatesBBox) {
    NTHUR::Net net("test", 0, 0, 0);
    net.add_pin(NTHUR::Net::Pin(0, 0, 0));
    net.add_pin(NTHUR::Net::Pin(10, 0, 0));
    net.add_pin(NTHUR::Net::Pin(0, 5, 0));
    // bbox = (10-0) + (5-0) = 15
    ASSERT_EQ(net.get_bboxSize(), 15);
    ASSERT_EQ(net.get_pinList().size(), 3u);
}

TEST(NetTest, SinglePinBBox) {
    NTHUR::Net net("single", 0, 0, 0);
    net.add_pin(NTHUR::Net::Pin(5, 5, 0));
    // bbox = (5-5) + (5-5) = 0
    ASSERT_EQ(net.get_bboxSize(), 0);
}

TEST(NetTest, SetName) {
    NTHUR::Net net("original", 0, 0, 0);
    net.set_name("renamed");
    ASSERT_EQ(net.get_name(), "renamed");
}

TEST(NetTest, CompNetOrdersByBBoxDesc) {
    NTHUR::Net small_net("small", 0, 0, 0);
    small_net.add_pin(NTHUR::Net::Pin(0, 0, 0));
    small_net.add_pin(NTHUR::Net::Pin(1, 1, 0));

    NTHUR::Net large_net("large", 1, 1, 0);
    large_net.add_pin(NTHUR::Net::Pin(0, 0, 0));
    large_net.add_pin(NTHUR::Net::Pin(10, 10, 0));

    // comp_net returns true if a has larger bbox
    ASSERT_TRUE(NTHUR::Net::comp_net(large_net, small_net));
    ASSERT_FALSE(NTHUR::Net::comp_net(small_net, large_net));
}

TEST(NetTest, ToString) {
    NTHUR::Net net("myNet", 5, 3, 2);
    net.add_pin(NTHUR::Net::Pin(1, 2, 0));
    std::string s = net.toString();
    ASSERT_FALSE(s.empty());
    ASSERT_NE(s.find("myNet"), std::string::npos);
    ASSERT_NE(s.find("5"), std::string::npos);
}

}  // namespace
