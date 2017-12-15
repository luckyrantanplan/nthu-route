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

    int par_ind;
    ParameterSet& parameter_set;
    RoutingParameters& routing_parameter;

    vector<Two_pin_element_2d> two_pin_list;

    int flute_mode;

    int fail_find_path_count;

    std::vector<Two_pin_element> all_two_pin_list;

    RoutingRegion& rr_map;

    int done_iter;
    double alpha;
    int total_overflow;

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
    vector<Vertex_flute_ptr> vertex_fl;
    EdgePlane<int> bboxRouteStateMap;

    double adjust_value;
    Congestion congestion;
    void insert_all_two_pin_list(Two_pin_element_2d& mn_path_2d);

    void printMemoryUsage(const char* msg);

    void init_2pin_list();
    void init_flute();
    void free_memory_con2d();
    void bbox_route(Two_pin_list_2d& list, const double value);
    Monotonic_element L_pattern_max_cong(const Coordinate_2d& c1, const Coordinate_2d& c2, Two_pin_element_2d& two_pin_L_path, int net_id);

    void L_pattern_route(const Coordinate_2d& c1, const Coordinate_2d& c2, Two_pin_element_2d& two_pin_L_path, int net_id);

    void update_congestion_map_remove_multipin_net(Two_pin_list_2d& list);

    void gen_FR_congestion_map();
    double compute_L_pattern_cost(int x1, int y1, int x2, int y2, int net_id);
    void find_saferange(Vertex_flute_ptr a, Vertex_flute_ptr b, int *low, int *high, int dir);
    void merge_vertex(Vertex_flute_ptr keep, Vertex_flute_ptr deleted);
    bool move_edge(Vertex_flute_ptr a, Vertex_flute_ptr b, int best_pos, int dir);
    void traverse_tree(double *ori_cost);
    void dfs_output_tree(Vertex_flute_ptr node, Tree *t);
    void edge_shifting(Tree *t);
    void output_2_pin_list();
    Construct_2d_tree(RoutingParameters & routingparam, ParameterSet & param, RoutingRegion & rr);
    void walkL(const Coordinate_2d& a, const Coordinate_2d& b, std::function<void(const Coordinate_2d& e1, const Coordinate_2d& e2)> f);

}
;

#endif /* _CONSTRUCT_2D_TREE_H_ */
