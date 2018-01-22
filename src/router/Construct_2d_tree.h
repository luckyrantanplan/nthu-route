#ifndef _CONSTRUCT_2D_TREE_H_
#define _CONSTRUCT_2D_TREE_H_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "flute4nthuroute.h"
#include "../grdb/EdgePlane.h"
#include "../misc/geometry.h"
#include "DataDef.h"
#include "MM_mazeroute.h"
#include "parameter.h"
#include "Post_processing.h"
#include "Range_router.h"

namespace spdlog {
class logger;
} /* namespace spdlog */

namespace NTHUR {

class Monotonic_element;

class Net;
class ParameterSet;
class RoutingParameters;
class RoutingRegion;

class Vertex_flute {
public:

    enum Type {
        PIN, STEINER, DELETED
    };

    Coordinate_2d c;
    Type type;   //PIN, STEINER, DELETED

    int visit;
    std::vector<Vertex_flute *> neighbor;

    Vertex_flute(int x, int y, Type type) :
            c { x, y }, type { type }, visit(0)/*, copy_ind(-1)*/{
    }

    //sort by x,y,pin,steiner
    static bool comp_vertex_fl(const Vertex_flute& a, const Vertex_flute& b);

    std::string toString() const {
        std::string s = "c: " + c.toString();

        s += " visit: " + std::to_string(visit);
        s += " neighbor: [";
        for (Vertex_flute * n : neighbor) {
            s += n->c.toString() + " ";
        }
        s += "]";
        return s;
    }

};

typedef Vertex_flute *Vertex_flute_ptr;

struct Construct_2d_tree {

    enum {
        HOR, VER
    };

    std::vector<Two_pin_element_2d> two_pin_list;

    EdgePlane<int> bboxRouteStateMap;
    const RoutingRegion& rr_map;

    int done_iter;

    int BOXSIZE_INC;
    std::vector<TreeFlute> net_flutetree;

    std::vector<bool> NetDirtyBit;
    Congestion& congestion;
    Multisource_multisink_mazeroute mazeroute_in_range;
    RangeRouter rangeRouter;
    Post_processing post_processing;
    /***********************
     * Global Variable End
     * ********************/

    std::vector<Two_pin_list_2d> net_2pin_list;      //store 2pin list of each net
    std::vector<Two_pin_list_2d> bbox_2pin_list;    //store bbox 2pin list of each net

    std::shared_ptr<spdlog::logger> log_sp;

    void init_2pin_list();
    void init_flute();
    void bbox_route(Two_pin_list_2d& list, const double value);

    Monotonic_element L_pattern_max_cong(const Coordinate_2d& c1, const Coordinate_2d& c2, Two_pin_element_2d& two_pin_L_path, int net_id);

    void L_pattern_route(const Coordinate_2d& c1, const Coordinate_2d& c2, Two_pin_element_2d& two_pin_L_path, int net_id);

    void gen_FR_congestion_map();
    double compute_L_pattern_cost(const Coordinate_2d& c1, const Coordinate_2d& c2, int net_id);
    void find_saferange(Vertex_flute& a, Vertex_flute& b, int *low, int *high, int dir);
    void merge_vertex(Vertex_flute& keep, Vertex_flute& deleted);
    bool move_edge(Vertex_flute& a, Vertex_flute& b, int best_pos, int dir);
    void traverse_tree(double& ori_cost, std::vector<Vertex_flute>& vertex_fl);
    void dfs_output_tree(Vertex_flute& node, int parent, TreeFlute& t);
    void edge_shifting(TreeFlute& t, int i);
    void output_2_pin_list();
    Construct_2d_tree(const RoutingParameters & routingparam, const RoutingRegion & rr, Congestion& congestion);
    void walkL(const Coordinate_2d& a, const Coordinate_2d& b, std::function<void(const Coordinate_2d& e1, const Coordinate_2d& e2)> f);

private:
    Vertex_flute_ptr findY(Vertex_flute& a, std::function<bool(const int& i, const int& j)> test);
    Vertex_flute_ptr findX(Vertex_flute& a, std::function<bool(const int& i, const int& j)> test);
    void move_edge_hor(Vertex_flute& a, int best_pos, Vertex_flute& b, Vertex_flute_ptr& overlap_a, std::function<bool(const int& i, const int& j)> test);
    void move_edge_ver(Vertex_flute& a, int best_pos, Vertex_flute& b, Vertex_flute_ptr& overlap_a, std::function<bool(const int& i, const int& j)> test);
}
;

} // namespace NTHUR

#endif /* _CONSTRUCT_2D_TREE_H_ */
