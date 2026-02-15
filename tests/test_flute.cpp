#include <gtest/gtest.h>

#include <cstdlib>

#include "flute/flute-ds.h"
#include "flute/flute-function.h"

namespace {

class FluteTest : public ::testing::Test {
  protected:
    static void SetUpTestSuite() { readLUT(); }
};

TEST_F(FluteTest, TwoPointWirelength) {
    DTYPE x[] = {0, 10};
    DTYPE y[] = {0, 5};
    DTYPE wl = flute_wl(2, x, y, ACCURACY);
    ASSERT_DOUBLE_EQ(wl, 15.0);
}

TEST_F(FluteTest, ThreePointWirelength) {
    DTYPE x[] = {0, 5, 10};
    DTYPE y[] = {0, 5, 0};
    DTYPE wl = flute_wl(3, x, y, ACCURACY);
    // Optimal RSMT: horizontal span 10 + vertical span 5 = 15
    ASSERT_DOUBLE_EQ(wl, 15.0);
}

TEST_F(FluteTest, CollinearPointsHorizontal) {
    DTYPE x[] = {0, 5, 10};
    DTYPE y[] = {0, 0, 0};
    DTYPE wl = flute_wl(3, x, y, ACCURACY);
    ASSERT_DOUBLE_EQ(wl, 10.0);
}

TEST_F(FluteTest, CollinearPointsVertical) {
    DTYPE x[] = {0, 0, 0};
    DTYPE y[] = {0, 5, 10};
    DTYPE wl = flute_wl(3, x, y, ACCURACY);
    ASSERT_DOUBLE_EQ(wl, 10.0);
}

TEST_F(FluteTest, TwoPointTree) {
    DTYPE x[] = {0, 10};
    DTYPE y[] = {0, 5};
    Tree t = flute(2, x, y, ACCURACY);
    ASSERT_EQ(t.deg, 2);
    ASSERT_DOUBLE_EQ(t.length, 15.0);
    ASSERT_NE(t.branch, nullptr);
    free(t.branch);
}

TEST_F(FluteTest, ThreePointTree) {
    DTYPE x[] = {0, 5, 10};
    DTYPE y[] = {0, 5, 0};
    Tree t = flute(3, x, y, ACCURACY);
    ASSERT_EQ(t.deg, 3);
    ASSERT_DOUBLE_EQ(t.length, 15.0);
    ASSERT_NE(t.branch, nullptr);
    free(t.branch);
}

TEST_F(FluteTest, TreeWirelengthConsistency) {
    DTYPE x[] = {0, 10, 5, 3};
    DTYPE y[] = {0, 10, 5, 8};
    Tree t = flute(4, x, y, ACCURACY);
    DTYPE wl = wirelength(t);
    ASSERT_DOUBLE_EQ(wl, t.length);
    free(t.branch);
}

TEST_F(FluteTest, WirelengthMatchesTree) {
    DTYPE x[] = {0, 10};
    DTYPE y[] = {0, 5};
    DTYPE wl = flute_wl(2, x, y, ACCURACY);
    Tree t = flute(2, x, y, ACCURACY);
    ASSERT_DOUBLE_EQ(wl, t.length);
    free(t.branch);
}

TEST_F(FluteTest, FourPointSquare) {
    DTYPE x[] = {0, 10, 10, 0};
    DTYPE y[] = {0, 0, 10, 10};
    DTYPE wl = flute_wl(4, x, y, ACCURACY);
    // Optimal RSMT for a 10x10 square: 30
    ASSERT_DOUBLE_EQ(wl, 30.0);
}

TEST_F(FluteTest, DuplicatePointsHandled) {
    DTYPE x[] = {5, 5, 10};
    DTYPE y[] = {5, 5, 10};
    Tree t = flute(3, x, y, ACCURACY);
    ASSERT_GT(t.deg, 0);
    ASSERT_NE(t.branch, nullptr);
    free(t.branch);
}

TEST_F(FluteTest, SinglePointWirelength) {
    // d=1 is a degenerate case, but d=2 with same point tests zero-length
    DTYPE x[] = {5, 5};
    DTYPE y[] = {5, 5};
    DTYPE wl = flute_wl(2, x, y, ACCURACY);
    ASSERT_DOUBLE_EQ(wl, 0.0);
}

TEST_F(FluteTest, LargerDegreeTree) {
    DTYPE x[] = {0, 2, 4, 6, 8};
    DTYPE y[] = {0, 3, 1, 4, 2};
    Tree t = flute(5, x, y, ACCURACY);
    ASSERT_EQ(t.deg, 5);
    ASSERT_GE(t.length, 0.0);
    DTYPE wl = wirelength(t);
    ASSERT_DOUBLE_EQ(wl, t.length);
    free(t.branch);
}

TEST_F(FluteTest, TreeBranchConnectivity) {
    DTYPE x[] = {0, 10, 5};
    DTYPE y[] = {0, 0, 5};
    Tree t = flute(3, x, y, ACCURACY);
    // Each branch should point to a valid index
    for (int i = 0; i < 2 * t.deg - 2; i++) {
        ASSERT_GE(t.branch[i].n, 0);
        ASSERT_LT(t.branch[i].n, 2 * t.deg - 2);
    }
    free(t.branch);
}

}  // namespace
