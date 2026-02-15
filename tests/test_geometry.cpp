#include <gtest/gtest.h>
#include <functional>
#include <string>

#include "../src/misc/geometry.h"

namespace {

// --- Coordinate_2d tests ---

TEST(Coordinate2dTest, DefaultConstruction) {
    NTHUR::Coordinate_2d c;
    ASSERT_EQ(c.x, 0);
    ASSERT_EQ(c.y, 0);
}

TEST(Coordinate2dTest, ParameterizedConstruction) {
    NTHUR::Coordinate_2d c(3, 7);
    ASSERT_EQ(c.x, 3);
    ASSERT_EQ(c.y, 7);
}

TEST(Coordinate2dTest, CopyConstruction) {
    NTHUR::Coordinate_2d a(5, 10);
    NTHUR::Coordinate_2d b(a);
    ASSERT_EQ(b.x, 5);
    ASSERT_EQ(b.y, 10);
}

TEST(Coordinate2dTest, CopyAssignment) {
    NTHUR::Coordinate_2d a(5, 10);
    NTHUR::Coordinate_2d b;
    b = a;
    ASSERT_EQ(b.x, 5);
    ASSERT_EQ(b.y, 10);
    // Ensure independence after copy
    a.x = 99;
    ASSERT_EQ(b.x, 5);
}

TEST(Coordinate2dTest, SelfAssignment) {
    NTHUR::Coordinate_2d a(3, 4);
    a = a;
    ASSERT_EQ(a.x, 3);
    ASSERT_EQ(a.y, 4);
}

TEST(Coordinate2dTest, Equality) {
    NTHUR::Coordinate_2d a(1, 2);
    NTHUR::Coordinate_2d b(1, 2);
    NTHUR::Coordinate_2d c(1, 3);
    ASSERT_TRUE(a == b);
    ASSERT_FALSE(a == c);
}

TEST(Coordinate2dTest, Inequality) {
    NTHUR::Coordinate_2d a(1, 2);
    NTHUR::Coordinate_2d b(1, 3);
    ASSERT_TRUE(a != b);
    ASSERT_FALSE(a != a);
}

TEST(Coordinate2dTest, Set) {
    NTHUR::Coordinate_2d c;
    c.set(4, 5);
    ASSERT_EQ(c.x, 4);
    ASSERT_EQ(c.y, 5);
}

TEST(Coordinate2dTest, SetFromOther) {
    NTHUR::Coordinate_2d a(7, 8);
    NTHUR::Coordinate_2d b;
    b.set(a);
    ASSERT_EQ(b.x, 7);
    ASSERT_EQ(b.y, 8);
}

TEST(Coordinate2dTest, Addition) {
    NTHUR::Coordinate_2d a(1, 2);
    NTHUR::Coordinate_2d b(3, 4);
    NTHUR::Coordinate_2d c = a + b;
    ASSERT_EQ(c.x, 4);
    ASSERT_EQ(c.y, 6);
}

TEST(Coordinate2dTest, IsAligned) {
    NTHUR::Coordinate_2d a(3, 5);
    NTHUR::Coordinate_2d b(3, 9);  // same x
    NTHUR::Coordinate_2d c(7, 5);  // same y
    NTHUR::Coordinate_2d d(7, 9);  // neither
    ASSERT_TRUE(a.isAligned(b));
    ASSERT_TRUE(a.isAligned(c));
    ASSERT_FALSE(a.isAligned(d));
}

TEST(Coordinate2dTest, ToString) {
    NTHUR::Coordinate_2d c(10, 20);
    ASSERT_EQ(c.toString(), "10,20");
}

TEST(Coordinate2dTest, DirArray) {
    auto dirs = NTHUR::Coordinate_2d::dir_array();
    ASSERT_EQ(dirs.size(), 4u);
    // FRONT(1,0), BACK(0,1), LEFT(-1,0), RIGHT(0,-1)
    ASSERT_EQ(dirs[0], NTHUR::Coordinate_2d(1, 0));
    ASSERT_EQ(dirs[1], NTHUR::Coordinate_2d(0, 1));
    ASSERT_EQ(dirs[2], NTHUR::Coordinate_2d(-1, 0));
    ASSERT_EQ(dirs[3], NTHUR::Coordinate_2d(0, -1));
}

TEST(Coordinate2dTest, HashUniqueness) {
    std::hash<NTHUR::Coordinate_2d> hasher;
    NTHUR::Coordinate_2d a(1, 2);
    NTHUR::Coordinate_2d b(2, 1);
    NTHUR::Coordinate_2d c(1, 2);
    // Same coordinates should hash equal
    ASSERT_EQ(hasher(a), hasher(c));
    // Different coordinates should (very likely) hash differently
    ASSERT_NE(hasher(a), hasher(b));
}

// --- Coordinate_3d tests ---

TEST(Coordinate3dTest, DefaultConstruction) {
    NTHUR::Coordinate_3d c;
    ASSERT_EQ(c.x, 0);
    ASSERT_EQ(c.y, 0);
    ASSERT_EQ(c.z, 0);
}

TEST(Coordinate3dTest, ParameterizedConstruction) {
    NTHUR::Coordinate_3d c(1, 2, 3);
    ASSERT_EQ(c.x, 1);
    ASSERT_EQ(c.y, 2);
    ASSERT_EQ(c.z, 3);
}

TEST(Coordinate3dTest, ConstructionFrom2d) {
    NTHUR::Coordinate_2d c2(5, 6);
    NTHUR::Coordinate_3d c3(c2, 7);
    ASSERT_EQ(c3.x, 5);
    ASSERT_EQ(c3.y, 6);
    ASSERT_EQ(c3.z, 7);
}

TEST(Coordinate3dTest, XYProjection) {
    NTHUR::Coordinate_3d c(3, 4, 5);
    NTHUR::Coordinate_2d xy = c.xy();
    ASSERT_EQ(xy.x, 3);
    ASSERT_EQ(xy.y, 4);
}

TEST(Coordinate3dTest, Set) {
    NTHUR::Coordinate_3d c;
    c.set(10, 20, 30);
    ASSERT_EQ(c.x, 10);
    ASSERT_EQ(c.y, 20);
    ASSERT_EQ(c.z, 30);
}

TEST(Coordinate3dTest, Equality) {
    NTHUR::Coordinate_3d a(1, 2, 3);
    NTHUR::Coordinate_3d b(1, 2, 3);
    NTHUR::Coordinate_3d c(1, 2, 4);
    ASSERT_TRUE(a == b);
    ASSERT_FALSE(a == c);
}

TEST(Coordinate3dTest, IsAligned) {
    NTHUR::Coordinate_3d a(1, 2, 3);
    NTHUR::Coordinate_3d b(1, 2, 5);  // same x,y
    NTHUR::Coordinate_3d c(1, 4, 3);  // same x,z
    NTHUR::Coordinate_3d d(5, 2, 3);  // same y,z
    NTHUR::Coordinate_3d e(5, 4, 5);  // none aligned
    ASSERT_TRUE(a.isAligned(b));
    ASSERT_TRUE(a.isAligned(c));
    ASSERT_TRUE(a.isAligned(d));
    ASSERT_FALSE(a.isAligned(e));
}

TEST(Coordinate3dTest, ToString) {
    NTHUR::Coordinate_3d c(1, 2, 3);
    ASSERT_EQ(c.toString(), "(1,2,3)");
}

// --- Rectangle tests ---

TEST(RectangleTest, Construction) {
    NTHUR::Coordinate_2d a(5, 10);
    NTHUR::Coordinate_2d b(1, 3);
    NTHUR::Rectangle r(a, b);
    // upLeft should be min, downRight should be max
    ASSERT_EQ(r.upLeft.x, 1);
    ASSERT_EQ(r.upLeft.y, 3);
    ASSERT_EQ(r.downRight.x, 5);
    ASSERT_EQ(r.downRight.y, 10);
}

TEST(RectangleTest, ConstructionReversed) {
    // Order shouldn't matter
    NTHUR::Coordinate_2d a(1, 3);
    NTHUR::Coordinate_2d b(5, 10);
    NTHUR::Rectangle r(a, b);
    ASSERT_EQ(r.upLeft.x, 1);
    ASSERT_EQ(r.upLeft.y, 3);
    ASSERT_EQ(r.downRight.x, 5);
    ASSERT_EQ(r.downRight.y, 10);
}

TEST(RectangleTest, ContainsPoint) {
    NTHUR::Rectangle r(NTHUR::Coordinate_2d(0, 0), NTHUR::Coordinate_2d(10, 10));
    ASSERT_TRUE(r.contains(NTHUR::Coordinate_2d(5, 5)));   // inside
    ASSERT_TRUE(r.contains(NTHUR::Coordinate_2d(0, 0)));   // corner
    ASSERT_TRUE(r.contains(NTHUR::Coordinate_2d(10, 10))); // corner
    ASSERT_TRUE(r.contains(NTHUR::Coordinate_2d(5, 0)));   // edge
    ASSERT_FALSE(r.contains(NTHUR::Coordinate_2d(11, 5))); // outside
    ASSERT_FALSE(r.contains(NTHUR::Coordinate_2d(-1, 5))); // outside
}

TEST(RectangleTest, ContainsRectangle) {
    NTHUR::Rectangle outer(NTHUR::Coordinate_2d(0, 0), NTHUR::Coordinate_2d(10, 10));
    NTHUR::Rectangle inner(NTHUR::Coordinate_2d(2, 2), NTHUR::Coordinate_2d(8, 8));
    NTHUR::Rectangle overlap(NTHUR::Coordinate_2d(5, 5), NTHUR::Coordinate_2d(15, 15));
    ASSERT_TRUE(outer.contains(inner));
    ASSERT_FALSE(outer.contains(overlap));
    ASSERT_FALSE(inner.contains(outer));
}

TEST(RectangleTest, Expand) {
    NTHUR::Rectangle r(NTHUR::Coordinate_2d(5, 5), NTHUR::Coordinate_2d(10, 10));
    r.expand(2);
    ASSERT_EQ(r.upLeft.x, 3);
    ASSERT_EQ(r.upLeft.y, 3);
    ASSERT_EQ(r.downRight.x, 12);
    ASSERT_EQ(r.downRight.y, 12);
}

TEST(RectangleTest, Clip) {
    NTHUR::Rectangle bound(NTHUR::Coordinate_2d(0, 0), NTHUR::Coordinate_2d(10, 10));
    NTHUR::Rectangle r(NTHUR::Coordinate_2d(-5, -5), NTHUR::Coordinate_2d(15, 15));
    bound.clip(r);
    ASSERT_EQ(r.upLeft.x, 0);
    ASSERT_EQ(r.upLeft.y, 0);
    ASSERT_EQ(r.downRight.x, 10);
    ASSERT_EQ(r.downRight.y, 10);
}

TEST(RectangleTest, ToString) {
    NTHUR::Rectangle r(NTHUR::Coordinate_2d(1, 2), NTHUR::Coordinate_2d(3, 4));
    std::string s = r.toString();
    ASSERT_FALSE(s.empty());
    ASSERT_NE(s.find("1"), std::string::npos);
    ASSERT_NE(s.find("4"), std::string::npos);
}

}  // namespace
