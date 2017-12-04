#ifndef INC_REOUTE_2PINNETS_H
#define INC_REOUTE_2PINNETS_H

#include <boost/multi_array.hpp>
#include <functional>
#include <vector>

#include "DataDef.h"
#include "Range_router.h"

class Two_pin_element_2d;
struct Construct_2d_tree;
class RoutingRegion;
struct RangeRouter;

class Point_fc {
public:
    Point_fc(int x = 0, int y = 0) :
            x(x), y(y) {
    }

public:
    int x;
    int y;
    std::vector<Two_pin_element_2d*> points;
};

struct ColorMap {

    //This color map is used by
    //bfs_for_find_two_pin_list()
    //for recording which tiles are traversed
    int traverse;

    //This color map is used by
    //determine_is_terminal_or_steiner_point()
    //for recording which tiles contains terminal
    int terminal;

    void set(int tra, int ter) {
        traverse = tra;
        terminal = ter;
    }
    ColorMap() :
            traverse { -1 }, terminal { -1 } {

    }
};
struct Route_2pinnets {

    boost::multi_array<Point_fc, 2> gridcell; //This is some kind of color map, for recording
    //which 2-pin net passed which gCell

    boost::multi_array<ColorMap, 2> colorMap;

    int dirTransferTable[4] = { 1, 0, 3, 2 }; //FRONT <-> BACK, LEFT <-> RIGHT
    //Used by determine_is_terminal_or_steiner_point()

    Construct_2d_tree& construct_2d_tree;
    RoutingRegion& rr_map;
    RangeRouter& rangerouter;
    Congestion& congestion;

    Route_2pinnets(Construct_2d_tree& construct_2d_tree, RangeRouter& rangerouter, Congestion& congestion);
    void route_all_2pin_net();
    void allocate_gridcell();

    void init_gridcell();

    void reset_c_map_used_net_to_one();
    void put_terminal_color_on_colormap(int net_id);
    int determine_is_terminal_or_steiner_point(int xx, int yy, int dir, int net_id);
    void bfs_for_find_two_pin_list(Coordinate_2d start_coor, int net_id, bool insert_to_branch);
    void reallocate_two_pin_list(bool insert_to_branch);

};

#endif //INC_REOUTE_2PINNETS_H
