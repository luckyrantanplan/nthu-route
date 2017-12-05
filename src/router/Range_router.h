#ifndef INC_RANGE_ROUTER_H
#define INC_RANGE_ROUTER_H

//#include "Route_2pinnets.h"

#include <boost/multi_array.hpp>
#include <array>
#include <vector>

#include "../misc/geometry.h"
#include "DataDef.h"
#include "MonotonicRouting.h"
#include "Route_2pinnets.h"

class Congestion;

#define INTERVAL_NUM 10
#define EXPAND_RANGE_SIZE 10
#define EXPAND_RANGE_INC 1

class Point_fc;
struct Construct_2d_tree;
struct Two_pin_element_2d;
class Range_element {
public:
    int x1;
    int y1;
    int x2;
    int y2;

public:
    Range_element(int x1, int y1, int x2, int y2) :
            x1(x1), y1(y1), x2(x2), y2(y2) {
    }
    ;

};
class Grid_edge_element {
public:
    Coordinate_2d grid;
    OrientationType dir;

public:
    Grid_edge_element(Coordinate_2d& grid, OrientationType dir) :
            grid(grid), dir(dir) {
    }
};
class Interval_element {
public:
    double begin_value;
    double end_value;
    std::vector<Grid_edge_element> grid_edge_vector;
};
struct RangeRouter {
public:

    struct ColorMap {
        //This color map is used by

        //expand_range()
        //for recording which edges have expanded
        //
        int expand;
        //This color map is used by
        //query_range_2pin()
        //for recording if all the 2-pin nets which
        //locate on the same tile are routed

        int routeState;

        ColorMap() :
                expand { -1 }, routeState { -1 } {
        }
        void set(const int iExpand, const int iRouteState) {
            expand = iExpand;
            routeState = iRouteState;
        }
    };

    std::vector<Range_element> range_vector;
    std::array<Interval_element, INTERVAL_NUM> interval_list;

    int total_twopin = 0;

    Construct_2d_tree& construct_2d_tree;
    boost::multi_array<ColorMap, 2> colorMap;
    boost::multi_array<int, 2> routeStateMap;
    Congestion& congestion;
    MonotonicRouting monotonicRouter;
    RangeRouter(Construct_2d_tree& construct_2d_tree, Congestion& congestion, bool monotonic_enable);
    void define_interval();
    void divide_grid_edge_into_interval();
    void specify_all_range(boost::multi_array<Point_fc, 2> & gridCell);

    bool double_equal(double a, double b);
    bool comp_grid_edge(const Grid_edge_element& a, const Grid_edge_element& b);

    void insert_to_interval(double cong_value, Coordinate_2d coor_2d, OrientationType dir);

    void expand_range(int x1, int y1, int x2, int y2, int interval_index);
    void range_router(Two_pin_element_2d& two_pin, int version);
    bool inside_range(int left_x, int bottom_y, int right_x, int top_y, Coordinate_2d& pt);
    void query_range_2pin(int left_x, int bottom_y, int right_x, int top_y, //
            std::vector<Two_pin_element_2d*>& twopin_list, boost::multi_array<Point_fc, 2>& gridCell);

private:

}
;

#endif //INC_RANGE_ROUTER_H
