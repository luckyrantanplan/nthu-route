#ifndef _CONSTRUCT_2D_TREE_H_
#define _CONSTRUCT_2D_TREE_H_

#include <boost/multi_array.hpp>
#include <array>
#include <functional>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "../flute/flute-ds.h"
#include "../grdb/EdgePlane.h"
#include "../misc/geometry.h"
#include "../util/traversemap.h"
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

    const std::array<std::array<int, 2>, 4> dir_array; //FRONT,BACK,LEFT,RIGHT

    vector<Two_pin_element_2d> two_pin_list;
    int two_pin_list_size;
    int flute_mode;

    double max_congestion_factor;
    int fail_find_path_count;

    boost::multi_array<Vertex_3d, 3> cur_map_3d;
    std::vector<Two_pin_element> all_two_pin_list;

    RoutingRegion& rr_map;
    int cur_iter;
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
    EdgeColorMap<int> bboxRouteStateMap;


    double adjust_value;

    void update_congestion_map_insert_two_pin_net(Two_pin_element_2d& element);
    void update_congestion_map_remove_two_pin_net(Two_pin_element_2d& element);
    void insert_all_two_pin_list(Two_pin_element_2d& mn_path_2d);

    inline bool comp_stn_2pin(const Two_pin_element_2d&a, const Two_pin_element_2d&b) {
        return (a.boxSize() > b.boxSize());
    }

    inline
    void printMemoryUsage(const char* msg) {
        std::cout << msg << std::endl;
        //for print out memory usage
        std::ifstream mem("/proc/self/status");
        std::string memory;
        for (unsigned i = 0; i < 13; ++i) {
            getline(mem, memory);
            if (i > 10) {
                std::cout << memory << std::endl;
            }
        }
    }




    int cal_total_wirelength();
    void init_3d_map();
    void output_3d_map();

    void init_2d_map();
    void allocate_coor_array();
    void init_2pin_list();
    void init_flute();
    void free_memory_con2d();
    void bbox_route(Two_pin_list_2d& list, const double value);

    void L_pattern_route(int x1, int y1, int x2, int y2, Two_pin_element_2d& two_pin_L_path, int net_id);

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
    Construct_2d_tree(RoutingParameters& routingparam, ParameterSet& param, RoutingRegion& rr);

};

#endif /* _CONSTRUCT_2D_TREE_H_ */
