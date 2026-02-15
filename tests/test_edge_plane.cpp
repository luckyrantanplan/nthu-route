#include <gtest/gtest.h>

#include "../src/grdb/EdgePlane.h"

namespace {

TEST(EdgePlaneTest, Construction) {
    NTHUR::EdgePlane<int> plane(NTHUR::Coordinate_2d(5, 5));
    ASSERT_GT(plane.num_elements(), 0u);
}

TEST(EdgePlaneTest, EastEdgeAccess) {
    NTHUR::EdgePlane<int> plane(NTHUR::Coordinate_2d(3, 3));
    NTHUR::Coordinate_2d c(0, 0);
    plane.east(c) = 42;
    ASSERT_EQ(plane.east(c), 42);
}

TEST(EdgePlaneTest, SouthEdgeAccess) {
    NTHUR::EdgePlane<int> plane(NTHUR::Coordinate_2d(3, 3));
    NTHUR::Coordinate_2d c(0, 0);
    plane.south(c) = 99;
    ASSERT_EQ(plane.south(c), 99);
}

TEST(EdgePlaneTest, EdgeBetweenVerticesHorizontal) {
    NTHUR::EdgePlane<int> plane(NTHUR::Coordinate_2d(3, 3));
    NTHUR::Coordinate_2d c1(0, 0);
    NTHUR::Coordinate_2d c2(1, 0);
    plane.edge(c1, c2) = 7;
    ASSERT_EQ(plane.edge(c1, c2), 7);
    // Reverse direction should give same edge
    ASSERT_EQ(plane.edge(c2, c1), 7);
}

TEST(EdgePlaneTest, EdgeBetweenVerticesVertical) {
    NTHUR::EdgePlane<int> plane(NTHUR::Coordinate_2d(3, 3));
    NTHUR::Coordinate_2d c1(0, 0);
    NTHUR::Coordinate_2d c2(0, 1);
    plane.edge(c1, c2) = 11;
    ASSERT_EQ(plane.edge(c1, c2), 11);
    ASSERT_EQ(plane.edge(c2, c1), 11);
}

TEST(EdgePlaneTest, IndependentEdges) {
    NTHUR::EdgePlane<int> plane(NTHUR::Coordinate_2d(3, 3));
    NTHUR::Coordinate_2d a(0, 0), b(1, 0), c(0, 1);
    plane.edge(a, b) = 10;
    plane.edge(a, c) = 20;
    ASSERT_EQ(plane.edge(a, b), 10);
    ASSERT_EQ(plane.edge(a, c), 20);
}

TEST(EdgePlaneTest, AllRange) {
    NTHUR::EdgePlane<int> plane(NTHUR::Coordinate_2d(2, 2));
    auto range = plane.all();
    // Verify we can iterate all elements
    int count = 0;
    for (auto it = range.begin(); it != range.end(); ++it) {
        ++count;
    }
    ASSERT_EQ(static_cast<std::size_t>(count), plane.num_elements());
}

}  // namespace
