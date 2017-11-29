#ifndef INC_LAYER_ASSIGNMENT_H
#define INC_LAYER_ASSIGNMENT_H

#include <array>
#include <memory>
#include <set>
#include <vector>

#include "../misc/geometry.h"
#include "Construct_2d_tree.h"

struct ans {
    int idx;
    int val;
    void update(int itar, int ival) {
        idx = itar;
        val = ival;
    }

};
struct DP_NODE {
    int val;
    Coordinate_3d *pi;
};
struct NET_NODE {
    int id;
    int val;
};
struct AVERAGE_NODE {
    int id;
    int times;
    int val;
    int vo_times;
    double average;
    int bends;
};
struct NET_INFO_NODE {
    int xy;
    int z;
    double val;
};

struct MULTIPIN_NET_NODE {
    Two_pin_list_2d two_pin_net_list;
};

struct PATH_NODE {
    char val;
    char edge[4];
};
struct KLAT_NODE {
    int val;
    int via_cost;
    int via_overflow;
    int pi_z;
};
struct OVERFLOW_NODE {
    std::array<std::shared_ptr<int>, 4> edge;
};
struct UNION_NODE {
    int pi;
    int sx, sy, bx, by;
    int num;
};
struct LENGTH_NODE {
    int xy;
    int z;
};
struct VIADENSITY_NODE {
    int cur;
    int max;
};
struct PATH_EDGE_3D {
    set<int> used_net;
};

struct PATH_VERTEX_3D {
    std::array<std::shared_ptr<PATH_EDGE_3D>, 6> edge_list;
};

struct ZLayerInfo {
    KLAT_NODE klat;
    VIADENSITY_NODE viadensity;
    Coordinate_3d coord_3d;
};

struct LayerInfo {
    PATH_NODE path;
    OVERFLOW_NODE overflow;
    std::vector<ZLayerInfo> zLayerInfo;

};

struct Layer_assignment {
    int tar = -2;
    char follow_prefer_direction;
    //enum {GREEDY, SHORT_PATH};
    int l_option;
    int max_xx, max_yy, max_zz, overflow_max, *prefer_idx;
    std::vector<std::vector<std::vector<Coordinate_3d>>> coord_3d_map;
    int i_router, i_test_case, i_order, i_method;
    const char temp_buf[1000] = "1000";

    NET_NODE *net_order;
    DP_NODE ***dp_map;
    std::vector<AVERAGE_NODE> average_order;
    std::vector<NET_INFO_NODE> net_info;
    std::vector<MULTIPIN_NET_NODE> multi_pin_net;
    std::vector<std::vector<PATH_NODE>> path_map;
    std::vector<std::vector<std::vector<KLAT_NODE>>> klat_map;
    std::vector<std::vector<OVERFLOW_NODE>> overflow_map;
    std::vector<UNION_NODE> group_set;
    std::array<LENGTH_NODE, 1000> length_count;
    std::vector<std::vector<std::vector<VIADENSITY_NODE>>> viadensity_map;
    std::vector<std::vector<std::vector<PATH_VERTEX_3D>>> path_map_3d;
    std::vector<std::vector<LayerInfo>> layerInfo_map;
    Construct_2d_tree& construct_2d_tree;
    std::vector<std::vector<std::vector<Vertex_3d>>>& cur_map_3d;

    const int plane_dir[4][2] = { { 0, 1 }, { 0, -1 }, { -1, 0 }, { 1, 0 } };   // F B L R
    const int cube_dir[6][3] = { { 0, 1, 0 }, { 0, -1, 0 }, { -1, 0, 0 }, { 1, 0, 0 }, { 0, 0, 1 }, { 0, 0, -1 } }; // F B L R U D
    int global_net_id, global_x, global_y, global_max_layer, global_pin_num, global_pin_cost = 0, global_xy_reduce = 0, global_BFS_xy = 0;
    int min_DP_val, min_DP_idx[4], max_DP_k, min_DP_k, min_DP_via_cost;
    int total_pin_number = 0;
    std::vector<std::vector<std::vector<int>>> BFS_color_map;
    int temp_global_pin_cost, temp_global_xy_cost, after_xy_cost;

    int is_used_for_BFS[1300][1300];

    int global_increase_vo;

    void print_max_overflow();
    void find_overflow_max();
    void initial_3D_coordinate_map();
    void initial_overflow_map();
    void malloc_space();
    void update_cur_map_for_klat_xy(int cur_idx, Coordinate_2d *start, Coordinate_2d *end, int net_id);
    void update_cur_map_for_klat_z(int pre_idx, int cur_idx, Coordinate_2d *start, int net_id);
    void update_path_for_klat(Coordinate_2d *start);
    void cycle_reduction(int x, int y);
    int preprocess(int net_id);
    void rec_count(int level, int val, int *count_idx);
    void DP(int x, int y, int z);
    bool in_cube_and_have_edge(int x, int y, int z, int dir, int net_id);
    bool have_child(int pre_x, int pre_y, int pre_z, int pre_dir, int net_id);
    void generate_output(int net_id);
    void klat(int net_id);
    int count_via_overflow_for_a_segment(int x, int y, int start, int end);
    void greedy_layer_assignment(int x, int y, int z);
    void greedy(int net_id);
    bool comp_temp_net_order(int p, int q);
    int backtrace(int n);
    void find_group(int max);
    void initial_BFS_color_map();
    void malloc_BFS_color_map();
    void calculate_wirelength();
    void erase_cur_map_3d();
    void multiply_viadensity_map_by_times(double times);
    void sort_net_order();
    void calculate_cap();
    void generate_all_output();
    Layer_assignment(const std::string& outputFileNamePtr, Construct_2d_tree& onstruct_2d_tree);

};

#endif //INC_LAYER_ASSIGNMENT_H
