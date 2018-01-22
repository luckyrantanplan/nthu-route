#ifndef INC_REOUTE_2PINNETS_H
#define INC_REOUTE_2PINNETS_H

#include <boost/multi_array.hpp>
#include <sys/types.h>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "../flute/flute-ds.h"
#include "../misc/geometry.h"
#include "DataDef.h"

namespace spdlog {
class logger;
} /* namespace spdlog */

namespace NTHUR {

class Congestion;

class Two_pin_element_2d;
struct Construct_2d_tree;
class RoutingRegion;
struct RangeRouter;

class Point_fc {
public:
    Point_fc(int x = 0, int y = 0) :
            x(x), y(y) {
    }

    void set(const int ix, const int iy) {
        x = ix;
        y = iy;
    }

    std::string toString() const {
        std::string s = "x:" + std::to_string(x);
        s += " y:" + std::to_string(y);
        s += " points:[";
        for (Two_pin_element_2d* twoPin : points) {
            s += "(" + twoPin->toString() + ") ";
        }
        s += "]";
        return s;
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

    struct ElementQueue {
        Coordinate_2d coor;
        Coordinate_2d parent;
        uint32_t index;

        ElementQueue(Coordinate_2d coor, Coordinate_2d parent, uint32_t index) :
                coor { coor }, parent { parent }, index { index } {
        }
    };

    struct BranchClass {
        double x;
        double y;   // starting point of the branch
        int n;        // index of neighbor

        BranchClass(Coordinate_2d c, int32_t n) :
                x { static_cast<double>(c.x) }, y { static_cast<double>(c.y) }, n { n } {
        }
    };

    enum PointType {
        oneDegreeTerminal, severalDegreeTerminal, oneDegreeNonterminal, steinerPoint, twoDegree
    };

    const RoutingRegion& rr_map;
    boost::multi_array<Point_fc, 2> gridcell; //This is some kind of color map, for recording
    //which 2-pin net passed which gCell

    boost::multi_array<ColorMap, 2> colorMap;

    int dirTransferTable[4] = { 1, 0, 3, 2 }; //FRONT <-> BACK, LEFT <-> RIGHT
    //Used by determine_is_terminal_or_steiner_point()

    Construct_2d_tree& construct_2d_tree;
    RangeRouter& rangerouter;
    Congestion& congestion;

    std::shared_ptr<spdlog::logger> log_sp;
    Route_2pinnets(Construct_2d_tree& construct_2d_tree, RangeRouter& rangerouter, Congestion& congestion);
    void route_all_2pin_net();
    void allocate_gridcell();

    void init_gridcell();

    void reset_c_map_used_net_to_one();
    void put_terminal_color_on_colormap(int net_id);
    Coordinate_2d determine_is_terminal_or_steiner_point(Coordinate_2d& c, Coordinate_2d& head, int net_id, PointType& pointType);
    void bfs_for_find_two_pin_list(Coordinate_2d start_coor, int net_id);
    void reallocate_two_pin_list();

private:
    void add_two_pin(int net_id, std::vector<Coordinate_2d>& path);
    void fillTree(int offset, int net_id);

};

} // namespace NTHUR

#endif //INC_REOUTE_2PINNETS_H
