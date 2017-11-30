#ifndef INC_RANGE_ROUTER_H
#define INC_RANGE_ROUTER_H

//#include "Route_2pinnets.h"

#include <vector>

#include "../grdb/plane.h"
#include "../util/traversemap.h"

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
    Coordinate_2d& grid;
    int dir;

public:
    Grid_edge_element(Coordinate_2d& grid, int dir) :
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

    std::vector<Range_element> range_vector;
    Interval_element interval_list[INTERVAL_NUM];

    int total_twopin = 0;
    int num_of_grid_edge = 0;
    double min_congestion = 0.;
    double max_congestion = 0.;
    double avg_congestion = 0.;
    int intervalCount;

    Construct_2d_tree& construct_2d_tree;
    boost::multi_array<int, 2> expandMap;   //This color map is used by

    //expand_range()
    //for recording which edges have expanded
    //
    boost::multi_array<int, 2> routeStateMap; //This color map is used by
    //query_range_2pin()
    //for recording if all the 2-pin nets which
    //locate on the same tile are routed

    RangeRouter(Construct_2d_tree& construct_2d_tree);

    void define_interval();
    void divide_grid_edge_into_interval();
    void specify_all_range(boost::multi_array<Point_fc, 2> & gridCell);

    bool double_equal(double a, double b);
    bool comp_grid_edge(const Grid_edge_element& a, const Grid_edge_element& b);

    void insert_to_interval(double cong_value, Coordinate_2d& coor_2d, int dir);

    void expand_range(int x1, int y1, int x2, int y2, int interval_index);
    void range_router(Two_pin_element_2d& two_pin);
    bool inside_range(int left_x, int bottom_y, int right_x, int top_y, Coordinate_2d& pt);
    void query_range_2pin(int left_x, int bottom_y, int right_x, int top_y, //
            std::vector<Two_pin_element_2d>& twopin_list, boost::multi_array<Point_fc, 2>& gridCell);

};

#endif //INC_RANGE_ROUTER_H
