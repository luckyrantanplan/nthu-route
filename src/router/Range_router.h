#ifndef INC_RANGE_ROUTER_H
#define INC_RANGE_ROUTER_H

//#include "Route_2pinnets.h"

#include <boost/multi_array.hpp>
#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "../misc/geometry.h"
#include "DataDef.h"
#include "MonotonicRouting.h"
#include "Route_2pinnets.h"

namespace spdlog {
class logger;
} /* namespace spdlog */

namespace NTHUR {

constexpr int INTERVAL_NUM = 10;
constexpr int EXPAND_RANGE_SIZE = 10;
constexpr int EXPAND_RANGE_INC = 1;

class Congestion;

class Point_fc;
struct Construct_2d_tree;
struct Two_pin_element_2d;

class Grid_edge_element {
public:
    Coordinate_2d grid;
    Coordinate_2d c2;

public:
    Grid_edge_element(Coordinate_2d& grid, Coordinate_2d& c2) :
            grid(grid), c2 { c2 } {
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

    std::vector<Rectangle> range_vector;
    std::array<Interval_element, INTERVAL_NUM> interval_list;

    int total_twopin = 0;

    Construct_2d_tree& construct_2d_tree;
    Congestion& congestion;
    boost::multi_array<ColorMap, 2> colorMap;
    MonotonicRouting monotonicRouter;

    std::shared_ptr<spdlog::logger> log_sp;

    RangeRouter(Construct_2d_tree& construct_2d_tree, Congestion& congestion, bool monotonic_enable);
    void define_interval();
    void divide_grid_edge_into_interval();
    void specify_all_range(boost::multi_array<Point_fc, 2> & gridCell);

    bool double_equal(double a, double b);
    bool comp_grid_edge(const Grid_edge_element& a, const Grid_edge_element& b);

    void insert_to_interval(Coordinate_2d coor_2d, Coordinate_2d c2);
    void walkFrame(const Rectangle& r, std::function<void(Coordinate_2d& i, Coordinate_2d& before)> accumulate);

    void expand_range(Coordinate_2d c1, Coordinate_2d c2, int interval_index);

    void range_router(Two_pin_element_2d& two_pin, int version);
    bool inside_range(int left_x, int bottom_y, int right_x, int top_y, Coordinate_2d& pt);
    void query_range_2pin(const Rectangle& r, std::vector<Two_pin_element_2d*>& twopin_list, boost::multi_array<Point_fc, 2>& gridCell);

private:
    std::string printIfBound(const Rectangle& r, const Rectangle& bound, const int interval_index, const Coordinate_2d& c1, const Coordinate_2d& c2) const;
    std::string print_interval() const;
}
;

} // namespace NTHUR

#endif //INC_RANGE_ROUTER_H
