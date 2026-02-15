#include <gtest/gtest.h>
#include <string>

#include "../src/router/DataDef.h"
#include "../src/misc/geometry.h"

namespace {

// --- Edge_2d tests ---

TEST(Edge2dTest, DefaultConstruction) {
    NTHUR::Edge_2d edge;
    ASSERT_FALSE(edge.isOverflow());
    ASSERT_EQ(edge.history, 1);
    ASSERT_TRUE(edge.used_net.empty());
}

TEST(Edge2dTest, OverflowDetection) {
    NTHUR::Edge_2d edge;
    edge.max_cap = 10;
    edge.cur_cap = 5;
    ASSERT_FALSE(edge.isOverflow());
    ASSERT_FALSE(edge.isFull());

    edge.cur_cap = 10;
    ASSERT_FALSE(edge.isOverflow());
    ASSERT_TRUE(edge.isFull());

    edge.cur_cap = 11;
    ASSERT_TRUE(edge.isOverflow());
    ASSERT_TRUE(edge.isFull());
}

TEST(Edge2dTest, OverUsage) {
    NTHUR::Edge_2d edge;
    edge.max_cap = 10;
    edge.cur_cap = 13;
    ASSERT_EQ(edge.overUsage(), 3);
}

TEST(Edge2dTest, LookupNet) {
    NTHUR::Edge_2d edge;
    ASSERT_FALSE(edge.lookupNet(42));
    edge.used_net[42] = 1;
    ASSERT_TRUE(edge.lookupNet(42));
    ASSERT_FALSE(edge.lookupNet(99));
}

TEST(Edge2dTest, Congestion) {
    NTHUR::Edge_2d edge;
    edge.max_cap = 10;
    edge.cur_cap = 5;
    ASSERT_DOUBLE_EQ(edge.congestion(), 0.5);
}

TEST(Edge2dTest, ToString) {
    NTHUR::Edge_2d edge;
    edge.max_cap = 10;
    edge.cur_cap = 5;
    std::string s = edge.toString();
    ASSERT_FALSE(s.empty());
}

// --- Two_pin_element_2d tests ---

TEST(TwoPinElement2dTest, DefaultConstruction) {
    NTHUR::Two_pin_element_2d elem;
    ASSERT_EQ(elem.net_id, 0);
    ASSERT_EQ(elem.done, -1);
    ASSERT_TRUE(elem.path.empty());
}

TEST(TwoPinElement2dTest, BoxSize) {
    NTHUR::Two_pin_element_2d elem;
    elem.pin1.set(0, 0);
    elem.pin2.set(5, 3);
    ASSERT_EQ(elem.boxSize(), 8);
}

TEST(TwoPinElement2dTest, BoxSizeNegative) {
    NTHUR::Two_pin_element_2d elem;
    elem.pin1.set(5, 3);
    elem.pin2.set(0, 0);
    ASSERT_EQ(elem.boxSize(), 8);
}

TEST(TwoPinElement2dTest, CompStn2pin) {
    NTHUR::Two_pin_element_2d a;
    a.pin1.set(0, 0);
    a.pin2.set(10, 10);

    NTHUR::Two_pin_element_2d b;
    b.pin1.set(0, 0);
    b.pin2.set(1, 1);

    // comp_stn_2pin returns true if a has larger boxSize
    ASSERT_TRUE(NTHUR::Two_pin_element_2d::comp_stn_2pin(a, b));
    ASSERT_FALSE(NTHUR::Two_pin_element_2d::comp_stn_2pin(b, a));
}

TEST(TwoPinElement2dTest, ToString) {
    NTHUR::Two_pin_element_2d elem;
    elem.pin1.set(1, 2);
    elem.pin2.set(3, 4);
    elem.net_id = 7;
    std::string s = elem.toString();
    ASSERT_FALSE(s.empty());
    ASSERT_NE(s.find("7"), std::string::npos);
}

}  // namespace
