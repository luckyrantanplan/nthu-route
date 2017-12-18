#ifndef _CONSTRUCT_2D_TREE_H_
#define _CONSTRUCT_2D_TREE_H_

#include <map>
#include <set>
#include <vector>

#include "../flute/flute-ds.h"
#include "../grdb/EdgePlane.h"
#include "Congestion.h"
#include "DataDef.h"
#include "MM_mazeroute.h"
#include "parameter.h"
#include "Post_processing.h"

class Net;
class ParameterSet;
class RoutingParameters;
class RoutingRegion;

using std::vector;
using std::set;
using std::map;

struct Construct_2d_tree {

    ParameterSet& parameter_set;
    RoutingParameters& routing_parameter;

    vector<Two_pin_element_2d> two_pin_list;

    EdgePlane<int> bboxRouteStateMap;
    RoutingRegion& rr_map;

    int done_iter;

    int BOXSIZE_INC;
    std::vector<Tree> net_flutetree;

    vector<bool> NetDirtyBit;

    Multisource_multisink_mazeroute mazeroute_in_range;
    Post_processing post_processing;
    /***********************
     * Global Variable End
     * ********************/

    vector<Two_pin_list_2d> net_2pin_list;      //store 2pin list of each net
    Tree global_flutetree;
    vector<Two_pin_list_2d> bbox_2pin_list;    //store bbox 2pin list of each net

    Congestion congestion;

    void printMemoryUsage(const char* msg);

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
    void dfs_output_tree(Vertex_flute& node, Tree& t);
    void edge_shifting(Tree& t);
    void output_2_pin_list();
    Construct_2d_tree(RoutingParameters & routingparam, ParameterSet & param, RoutingRegion & rr);
    void walkL(const Coordinate_2d& a, const Coordinate_2d& b, std::function<void(const Coordinate_2d& e1, const Coordinate_2d& e2)> f);

private:
    Vertex_flute_ptr findY(Vertex_flute& a, std::function<bool(const int& i, const int& j)> test);
    Vertex_flute_ptr findX(Vertex_flute& a, std::function<bool(const int& i, const int& j)> test);
    void move_edge_hor(Vertex_flute& a, int best_pos, Vertex_flute& b, Vertex_flute_ptr& overlap_a, std::function<bool(const int& i, const int& j)> test);
    void move_edge_ver(Vertex_flute& a, int best_pos, Vertex_flute& b, Vertex_flute_ptr& overlap_a, std::function<bool(const int& i, const int& j)> test);
}
;

#endif /* _CONSTRUCT_2D_TREE_H_ */
