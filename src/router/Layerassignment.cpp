#include <sys/time.h>
#include <unistd.h>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "../grdb/plane.h"
#include "../grdb/RoutingComponent.h"
#include "../grdb/RoutingRegion.h"
#include "Layerassignment.h"
#include <algorithm>
#include <time.h>
#include <queue>
#include "Construct_2d_tree.h"

#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)
//#define FOLLOW_PREFER
#define VIA_DENSITY	 // new experiment 2007/09/27
//#define REDUCE_VIA_DENSITY	// new experiment 2007/09/29
#define CHECK_PREFER
#define MAX_OVERFLOW_CONSTRAINT
#define FROM2D
#define ALLOUTPUT

using namespace std;
using namespace Jm;

#ifdef CHECK_PREFER
char prefer_direction[6][2] = { 0 };
#endif

void Layer_assignment::print_max_overflow() {
    int i, j, k, lines = 0;
    int max = 0;
    int sum = 0, temp_sum;
    //int overflow_val;
    //int x, y;

    for (i = 1; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j) {
            temp_sum = 0;
            for (k = 0; k < max_zz; ++k) {
                temp_sum += cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap;
                if (cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap > cur_map_3d[i][j][k].edge_list[LEFT]->max_cap)	// overflow
                        {
                    if (cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap - cur_map_3d[i][j][k].edge_list[LEFT]->max_cap > max) {

                        max = cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap - cur_map_3d[i][j][k].edge_list[LEFT]->max_cap;
                    }
                    sum += (cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap - cur_map_3d[i][j][k].edge_list[LEFT]->max_cap);
                    lines++;
                }
            }
        }
    for (i = 0; i < max_xx; ++i)
        for (j = 1; j < max_yy; ++j) {
            temp_sum = 0;
            for (k = 0; k < max_zz; ++k) {
                temp_sum += cur_map_3d[i][j][k].edge_list[BACK]->cur_cap;
                if (cur_map_3d[i][j][k].edge_list[BACK]->cur_cap > cur_map_3d[i][j][k].edge_list[BACK]->max_cap)	// overflow
                        {
                    if (cur_map_3d[i][j][k].edge_list[BACK]->cur_cap - cur_map_3d[i][j][k].edge_list[BACK]->max_cap > max) {

                        max = cur_map_3d[i][j][k].edge_list[BACK]->cur_cap - cur_map_3d[i][j][k].edge_list[BACK]->max_cap;
                    }
                    sum += (cur_map_3d[i][j][k].edge_list[BACK]->cur_cap - cur_map_3d[i][j][k].edge_list[BACK]->max_cap);
                    lines++;
                }
            }
        }
    printf("3D # of overflow = %d\n", sum);
    printf("3D max overflow = %d\n", max);
    printf("3D overflow edge number = %d\n", lines);
}

void Layer_assignment::find_overflow_max() {
    int i, j;

    overflow_max = 0;
    for (i = 1; i < max_xx; ++i) {
        for (j = 0; j < max_yy; ++j) {

            Edge_2d& edge = construct_2d_tree.congestionMap2d->edge(i, j, DIR_WEST);

            if (edge.overUsage() > overflow_max) {
                overflow_max = edge.overUsage();
            }
        }
    }
    for (i = 0; i < max_xx; ++i) {
        for (j = 1; j < max_yy; ++j) {
            Edge_2d& edge = construct_2d_tree.congestionMap2d->edge(i, j, DIR_SOUTH);

            if (edge.overUsage() > overflow_max) {
                overflow_max = edge.overUsage();
            }
        }
    }
    printf("2D maximum overflow = %d\n", overflow_max);
#ifdef MAX_OVERFLOW_CONSTRAINT
#ifdef FOLLOW_PREFER
    if (follow_prefer_direction == 1)
    {
        if (overflow_max % (max_zz / 2))
        overflow_max = ((overflow_max / (max_zz / 2)) << 1) + 2;
        else
        overflow_max = ((overflow_max / (max_zz / 2)) << 1);
    }
    else
    {
        if (overflow_max % max_zz)
        overflow_max = ((overflow_max / max_zz) << 1) + 2;
        else
        overflow_max = ((overflow_max / max_zz) << 1);
    }
#else
    if (overflow_max % max_zz)
        overflow_max = ((overflow_max / max_zz) << 1) + 2;
    else
        overflow_max = ((overflow_max / max_zz) << 1);
#endif
#else
    overflow_max <<= 1;
#endif
    printf("overflow max = %d\n", overflow_max);
}

void Layer_assignment::initial_3D_coordinate_map() {

    layerInfo_map.resize(max_xx);

    for (std::vector<LayerInfo>& vi : layerInfo_map) {
        vi.resize(max_yy);
        for (LayerInfo& vj : vi) {
            vj.zLayerInfo.resize(max_zz);
        }
    }
    for (int i = 0; i < max_xx; ++i) {
        for (int j = 0; j < max_yy; ++j) {
            for (int k = 0; k < max_zz; ++k) {
                layerInfo_map[i][j].zLayerInfo[k].coord_3d = Coordinate_3d(i, j, k);
            }
        }
    }
}

void Layer_assignment::initial_overflow_map() {

    for (int i = 1; i < max_xx; ++i) {
        for (int j = 0; j < max_yy; ++j) {
            OVERFLOW_NODE& overflow = layerInfo_map[i][j].overflow;
            *(overflow.edge[LEFT]) = (construct_2d_tree.congestionMap2d->edge(i, j, DIR_WEST).overUsage() << 1);
            *(overflow.edge[BACK]) = (construct_2d_tree.congestionMap2d->edge(i, j, DIR_SOUTH).overUsage() << 1);
        }
    }

}

void Layer_assignment::malloc_space() {
    layerInfo_map.resize(max_xx);
    for (std::vector<LayerInfo>& vi : layerInfo_map) {
        vi.resize(max_yy);
        for (LayerInfo& vj : vi) {
            vj.zLayerInfo.resize(max_zz);
            for (ZLayerInfo& vk : vj.zLayerInfo) {
                vk.viadensity.cur = 0;
                vk.viadensity.max = 0;
            }
            vj.path.val = 0;  // non-visited
            for (int k = 0; k < 4; ++k) {
                vj.path.edge[k] = 0;
            }
        }
    }

    for (int i = 1; i < max_xx; ++i) {
        for (int j = 0; j < max_yy; ++j) {
            OVERFLOW_NODE& overflow = layerInfo_map[i][j].overflow;
            std::shared_ptr<int> temp = make_shared<int>();
            overflow.edge[LEFT] = temp;
            layerInfo_map[i - 1][j].overflow.edge[RIGHT] = temp;
        }
    }
    for (int i = 0; i < max_xx; ++i) {
        for (int j = 1; j < max_yy; ++j) {
            std::shared_ptr<int> temp = make_shared<int>();
            overflow.edge[BACK] = temp;
            layerInfo_map[i][j - 1].overflow.edge[FRONT] = temp;
        }
    }

}

void Layer_assignment::update_cur_map_for_klat_xy(int cur_idx, Coordinate_2d *start, Coordinate_2d *end, int net_id) {
    int dir_idx;

    if (start->x != end->x && start->y == end->y) {
        dir_idx = ((end->x > start->x) ? LEFT : RIGHT);
        cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->used_net[net_id]++;
        cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->cur_cap = (cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->used_net.size() << 1);	// need check
        if (cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->cur_cap > cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->max_cap)	// need check
                {
            *(overflow_map[end->x][end->y].edge[dir_idx]) -= 2;
        }
    } else if (start->y != end->y && start->x == end->x) {
        dir_idx = ((end->y > start->y) ? BACK : FRONT);
        cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->used_net[net_id]++;
        cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->cur_cap = (cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->used_net.size() << 1);	// need check
        if (cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->cur_cap > cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->max_cap)  // need check
                {
            *(overflow_map[end->x][end->y].edge[dir_idx]) -= 2;
        }
    } else if (start->x != end->x && start->y != end->y) {
        printf("%d %d %d %d\n", start->x, start->y, end->x, end->y);
        puts("ERROR!!!");
    }
}

void Layer_assignment::update_cur_map_for_klat_z(int pre_idx, int cur_idx, Coordinate_2d *start, int net_id) {
    int i, j, k, z_dir, dir_idx;

    if (pre_idx != cur_idx) {
        if (cur_idx > pre_idx) {
            z_dir = 1;
            dir_idx = UP;
        } else	// cur_idx < pre_idx
        {
            z_dir = -1;
            dir_idx = DOWN;
        }
        i = start->x;
        j = start->y;
        for (k = pre_idx; k != cur_idx; k += z_dir) {
            cur_map_3d[i][j][k].edge_list[dir_idx]->used_net[net_id]++;
            if (z_dir == 1)
                ++viadensity_map[i][j][k].cur;
            else
                ++viadensity_map[i][j][k - 1].cur;
        }
    }
}

void Layer_assignment::update_path_for_klat(Coordinate_2d *start) {
    int i, x, y, z_min, z_max, pin_num = 0;
    queue<std::reference_wrapper<Coordinate_3d>> q;

    // BFS
    q.push(coord_3d_map[start->x][start->y][0]);	// enqueue
    while (!q.empty()) {
        Coordinate_3d& temp = q.front();
        if (path_map[temp.x][temp.y].val == 2)	// a pin
                {
            pin_num++;
            z_min = 0;
        } else
            z_min = temp.z;
        z_max = temp.z;
        for (i = 0; i < 4; ++i)
            if (path_map[temp.x][temp.y].edge[i] == 1)	// check leagal
                    {
                x = temp.x + plane_dir[i][0];
                y = temp.y + plane_dir[i][1];
                if (path_map[x][y].val == 0)
                    puts("ERROR");
                if (klat_map[x][y][temp.z].pi_z > z_max)
                    z_max = klat_map[x][y][temp.z].pi_z;
                if (klat_map[x][y][temp.z].pi_z < z_min)
                    z_min = klat_map[x][y][temp.z].pi_z;
                update_cur_map_for_klat_xy(klat_map[x][y][temp.z].pi_z, &coor_array[temp.x][temp.y], &coor_array[x][y], global_net_id);
                q.push(coord_3d_map[x][y][klat_map[x][y][temp.z].pi_z]);	// enqueue
            }
        update_cur_map_for_klat_z(z_min, z_max, &coor_array[temp.x][temp.y], global_net_id);
        path_map[temp.x][temp.y].val = 0;	// visited
        q.pop();	// dequeue
    }
    if (pin_num != global_pin_num)
        printf("net : %d, pin number error %d vs %d\n", global_net_id, pin_num, global_pin_num);
}

void Layer_assignment::cycle_reduction(int x, int y) {
    int i, idx, temp_x, temp_y;

    for (i = 0; i < 4; ++i)
        if (path_map[x][y].edge[i] == 1)
            cycle_reduction(x + plane_dir[i][0], y + plane_dir[i][1]);
    for (idx = 0; idx < 4 && path_map[x][y].edge[idx] == 0; ++idx)
        ;
    if (idx == 4 && path_map[x][y].val == 1)	// a leaf && it is not a pin
            {
        path_map[x][y].val = 0;
        for (i = 0; i < 4; ++i) {
            temp_x = x + plane_dir[i][0];
            temp_y = y + plane_dir[i][1];
            if (temp_x >= 0 && temp_x < max_xx && temp_y >= 0 && temp_y < max_yy) {
                if (i == 0 && path_map[temp_x][temp_y].edge[1] == 1)
                    path_map[temp_x][temp_y].edge[1] = 0;
                else if (i == 1 && path_map[temp_x][temp_y].edge[0] == 1)
                    path_map[temp_x][temp_y].edge[0] = 0;
                else if (i == 2 && path_map[temp_x][temp_y].edge[3] == 1)
                    path_map[temp_x][temp_y].edge[3] = 0;
                else if (i == 3 && path_map[temp_x][temp_y].edge[2] == 1)
                    path_map[temp_x][temp_y].edge[2] = 0;
            }
        }
    }
}

int Layer_assignment::preprocess(int net_id) {
    int i, k, x, y, pin_counter = 1;
    queue<std::reference_wrapper<Coordinate_2d> > q;

    const PinptrList* pin_list = &rr_map->get_nPin(net_id);
#ifdef POSTPROCESS
    int xy_len = 0;
#endif

    after_xy_cost = 0;
    for (i = 0; i < global_pin_num; ++i) {
        x = (*pin_list)[i]->get_tileX();
        y = (*pin_list)[i]->get_tileY();
        path_map[x][y].val = -2;	// pin
    }
    // BFS speed up
    x = (*pin_list)[0]->get_tileX();
    y = (*pin_list)[0]->get_tileY();
    path_map[x][y].val = 2;	// visited
    //initial klat_map[x][y][k]
    for (k = 0; k < max_zz; ++k) {
        klat_map[x][y][k].val = -1;
    }
    q.push(coor_array[x][y]);	// enqueue
    while (!q.empty()) {
        Coordinate_2d& temp = q.front();
        for (i = 0; i < 4; ++i) {
            x = temp.x + plane_dir[i][0];
            y = temp.y + plane_dir[i][1];
            if (x >= 0 && x < max_xx && y >= 0 && y < max_yy && congestionMap2d->edge(temp.x, temp.y, i).lookupNet(net_id)) {
                if (path_map[x][y].val == 0 || path_map[x][y].val == -2) {
                    ++after_xy_cost;
#ifdef POSTPROCESS
                    ++xy_len;
#endif
                    global_BFS_xy++;
                    path_map[temp.x][temp.y].edge[i] = 1;
                    if (path_map[x][y].val == 0)
                        path_map[x][y].val = 1;	// visited node
                    else	// path_map[x][y].val == 2
                    {
                        path_map[x][y].val = 2;	// visited pin
                        ++pin_counter;
                    }
                    // initial klat_map[x][y][k]
                    for (k = 0; k < max_zz; ++k) {
                        klat_map[x][y][k].val = -1;
                    }
#ifndef VIA_DENSITY
                    if (max_layer < max_zz)
                    {
                        if (congestionMap2d->edge(temp.x, temp.y, i).cur_cap < congestionMap2d->edge(temp.x, temp.y, i).max_cap)
                        {
                            temp_cap = (congestionMap2d->edge(temp.x, temp.y, i).used_net.size() << 1);
                            for(k = 0; k < max_zz && temp_cap > 0; ++k)
                            if (cur_map_3d[temp.x][temp.y][k].edge_list[i]->max_cap > 0)
                            temp_cap -= cur_map_3d[temp.x][temp.y][k].edge_list[i]->max_cap;
                            if (k == max_zz)
                            max_layer = max_zz;
                            else if (k + 1 > max_layer)
                            max_layer = k + 1;
                        }
                        else
                        max_layer = max_zz;
                    }
#endif
                    q.push(coor_array[x][y]);	// enqueue
                } else
                    path_map[temp.x][temp.y].edge[i] = 0;
            } else
                path_map[temp.x][temp.y].edge[i] = 0;
        }
        q.pop();	// dequeue
    }
    cycle_reduction((*pin_list)[0]->get_tileX(), (*pin_list)[0]->get_tileY());
#ifdef POSTPROCESS
    net_info[net_id].xy = xy_len;
#endif
#ifdef VIA_DENSITY
    return max_zz;
#else
    return max_layer;
#endif
}

void Layer_assignment::rec_count(int level, int val, int *count_idx) {
    int k, temp_x, temp_y, temp_val, temp_via_cost;
    int temp_idx[4];
    int min_k, max_k, temp_min_k, temp_max_k;

    if (level < 4) {
        if (val <= min_DP_val) {
            for (k = 0; k < level; ++k)
                temp_idx[k] = count_idx[k];
            if (path_map[global_x][global_y].edge[level] == 1) {
                temp_x = global_x + plane_dir[level][0];
                temp_y = global_y + plane_dir[level][1];
                for (k = 0; k < global_max_layer; ++k)	// use global_max_layer to substitue max_zz
                    if (klat_map[temp_x][temp_y][k].val >= 0) {
                        temp_idx[level] = k;
                        rec_count(level + 1, val + klat_map[temp_x][temp_y][k].val, temp_idx);
                    }
            } else {
                temp_idx[level] = -1;
                rec_count(level + 1, val, temp_idx);
            }
        }
    } else	// level == 4
    {
        min_k = min_DP_k;
        max_k = max_DP_k;
        for (k = temp_via_cost = 0; k < 4; ++k)
            if (count_idx[k] != -1) {
                if (count_idx[k] < min_k)
                    min_k = count_idx[k];
                if (count_idx[k] > max_k)
                    max_k = count_idx[k];
                temp_via_cost += klat_map[global_x + plane_dir[k][0]][global_y + plane_dir[k][1]][count_idx[k]].via_cost;
            }
        temp_via_cost += ((max_k - min_k) * via_cost);
#ifdef VIA_DENSITY
        for (k = min_k, temp_val = val; k < max_k; ++k)
            if (viadensity_map[global_x][global_y][k].cur >= viadensity_map[global_x][global_y][k].max)
                ++temp_val;
#else
        temp_val = temp_via_cost;
#endif
        if (temp_val < min_DP_val) {
            min_DP_val = temp_val;
            min_DP_via_cost = temp_via_cost;
            for (k = 0; k < 4; ++k)
                min_DP_idx[k] = count_idx[k];
        } else if (temp_val == min_DP_val) {
#ifdef VIA_DENSITY
            if (temp_via_cost < min_DP_via_cost) {
                min_DP_via_cost = temp_via_cost;
                for (k = 0; k < 4; ++k)
                    min_DP_idx[k] = count_idx[k];
            } else if (temp_via_cost == min_DP_via_cost) {
                temp_min_k = min_DP_k;
                temp_max_k = max_DP_k;
                for (k = 0; k < 4; ++k)
                    if (min_DP_idx[k] != -1) {
                        if (min_DP_idx[k] < temp_min_k)
                            temp_min_k = min_DP_idx[k];
                        if (min_DP_idx[k] > temp_max_k)
                            temp_max_k = min_DP_idx[k];
                    }
                if (max_k > temp_max_k || min_k > temp_min_k)
                    for (k = 0; k < 4; ++k)
                        min_DP_idx[k] = count_idx[k];
            }
#else
            temp_min_k = min_DP_k;
            temp_max_k = max_DP_k;
            for(k = 0; k < 4; ++k)
            if (min_DP_idx[k] != -1)
            {
                if (min_DP_idx[k] < temp_min_k)
                temp_min_k = min_DP_idx[k];
                if (min_DP_idx[k] > temp_max_k)
                temp_max_k = min_DP_idx[k];
            }
            if (max_k > temp_max_k || min_k > temp_min_k)
            for(k = 0; k < 4; ++k)
            min_DP_idx[k] = count_idx[k];
#endif
        }
    }
}

void Layer_assignment::DP(int x, int y, int z) {
    int i, k, temp_x, temp_y;
    bool is_end;
    int count_idx[4];

    if (klat_map[x][y][z].val == -1)	// non-traversed
            {
        is_end = true;
        for (i = 0; i < 4; ++i)	// direction
                {
            if (path_map[x][y].edge[i] == 1)	// check leagal
                    {
                is_end = false;
                temp_x = x + plane_dir[i][0];
                temp_y = y + plane_dir[i][1];
                if (*(overflow_map[x][y].edge[i]) <= 0)	// this edge could not happen overflow
                    for (k = 0; k < global_max_layer; ++k)	// use global_max_layer to substitute max_zz
                            {
                        if (cur_map_3d[x][y][k].edge_list[i]->cur_cap < cur_map_3d[x][y][k].edge_list[i]->max_cap)	// pass without overflow
                            DP(temp_x, temp_y, k);
                    }
                else
                    // this edge could happen overflow
                    for (k = 0; k < global_max_layer; ++k)	// use global_max_layer to substitute max_zz
                            {
                        if ((cur_map_3d[x][y][k].edge_list[i]->cur_cap - cur_map_3d[x][y][k].edge_list[i]->max_cap) < overflow_max)	// overflow, but doesn't over the overflow_max
#ifdef FOLLOW_PREFER
                        if (/*follow_prefer_direction == 0 || (follow_prefer_direction == 1 && */(((i == 0 || i == 1) && prefer_direction[k][1] == 1) || ((i == 2 || i == 3) && prefer_direction[k][0] == 1)))//)
                        DP(temp_x, temp_y, k);
#else
                            DP(temp_x, temp_y, k);
#endif
                    }
            }
        }
        if (is_end == true) {
            klat_map[x][y][z].via_cost = via_cost * z;
#ifdef VIA_DENSITY
            for (k = klat_map[x][y][z].via_overflow = 0; k < z; ++k)
                if (viadensity_map[x][y][k].cur >= viadensity_map[x][y][k].max)
                    ++klat_map[x][y][z].via_overflow;
            klat_map[x][y][z].val = klat_map[x][y][z].via_overflow;
#else
            klat_map[x][y][z].val = klat_map[x][y][z].via_cost;
#endif
        } else	// is_end == false
        {
            // special purpose
            min_DP_val = 1000000000;
            min_DP_via_cost = 1000000000;
            global_x = x;
            global_y = y;
            max_DP_k = z;
            if (path_map[x][y].val == 1)	// not a pin
                min_DP_k = z;
            else
                // pin
                min_DP_k = 0;
            for (i = 0; i < 4; ++i)
                min_DP_idx[i] = -1;
            rec_count(0, 0, count_idx);
            klat_map[x][y][z].via_cost = min_DP_via_cost;
#ifdef VIA_DENSITY
            klat_map[x][y][z].via_overflow = min_DP_val;
#endif
            klat_map[x][y][z].val = min_DP_val;
            for (i = 0; i < 4; ++i)	// direction
                if (min_DP_idx[i] >= 0) {
                    temp_x = x + plane_dir[i][0];
                    temp_y = y + plane_dir[i][1];
                    klat_map[temp_x][temp_y][z].pi_z = min_DP_idx[i];
                }
        }
    }
}

bool Layer_assignment::in_cube_and_have_edge(int x, int y, int z, int dir, int net_id) {
    // F B L R U D
    int anti_dir[6] = { 1, 0, 3, 2, 5, 4 };
    // B F R L D U

    if (x >= 0 && x < max_xx && y >= 0 && y < max_yy && z >= 0 && z < max_zz) {
        if (cur_map_3d[x][y][z].edge_list[anti_dir[dir]]->used_net.find(net_id) != cur_map_3d[x][y][z].edge_list[anti_dir[dir]]->used_net.end()) {
            return true;
        } else
            return false;
    } else
        return false;
}

bool Layer_assignment::have_child(int pre_x, int pre_y, int pre_z, int pre_dir, int net_id) {
    int dir, x, y, z;
    int anti_dir[6] = { 1, 0, 3, 2, 5, 4 };

    for (dir = 0; dir < 6; ++dir)
        if (dir != pre_dir && dir != anti_dir[pre_dir]) {
            x = pre_x + cube_dir[dir][0];
            y = pre_y + cube_dir[dir][1];
            z = pre_z + cube_dir[dir][2];
            if (in_cube_and_have_edge(x, y, z, dir, net_id) == true && BFS_color_map[x][y][z] != net_id)
                return true;
        }
    return false;
}

void Layer_assignment::generate_output(int net_id) {
    int i, j;
    const PinptrList* pin_list = &rr_map->get_nPin(net_id);
    const char *p;
    queue<std::reference_wrapper<Coordinate_3d>> q;
    int dir;

    Coordinate_3d start;
    Coordinate_3d end;

    int xDetailShift = rr_map->get_llx() + (rr_map->get_tileWidth() >> 1);
    int yDetailShift = rr_map->get_lly() + (rr_map->get_tileHeight() >> 1);

    // the beginning of a net of output file
    p = rr_map->get_netName(net_id);
    printf("%s", p);
    for (i = 0; p[i] && (p[i] < '0' || p[i] > '9'); ++i)
        ;
    printf(" %s\n", p + i);
    // BFS
    q.push(coord_3d_map[(*pin_list)[0]->get_tileX()][(*pin_list)[0]->get_tileY()][0]);	// enqueue
    Coordinate_3d& temp = q.front();
    BFS_color_map[temp.x][temp.y][temp.z] = net_id;
    while (!q.empty()) {
        temp = q.front();
        for (dir = 0; dir < 6; dir += 2) {
            int dirPlusOne = dir + 1;
            start.x = end.x = temp.x;
            start.y = end.y = temp.y;
            start.z = end.z = temp.z;

            for (i = 1;; ++i) {
                start.x += cube_dir[dir][0];
                start.y += cube_dir[dir][1];
                start.z += cube_dir[dir][2];
                if (in_cube_and_have_edge(start.x, start.y, start.z, dir, net_id) == true && BFS_color_map[start.x][start.y][start.z] != net_id) {
                    BFS_color_map[start.x][start.y][start.z] = net_id;
                    if (have_child(start.x, start.y, start.z, dir, net_id) == true)
                        q.push(coord_3d_map[start.x][start.y][start.z]);	// enqueue
                } else {
                    start.x -= cube_dir[dir][0];
                    start.y -= cube_dir[dir][1];
                    start.z -= cube_dir[dir][2];
                    break;
                }
            }
            for (j = 1;; ++j) {
                end.x += cube_dir[dirPlusOne][0];
                end.y += cube_dir[dirPlusOne][1];
                end.z += cube_dir[dirPlusOne][2];
                if (in_cube_and_have_edge(end.x, end.y, end.z, dirPlusOne, net_id) == true && BFS_color_map[end.x][end.y][end.z] != net_id) {
                    BFS_color_map[end.x][end.y][end.z] = net_id;
                    if (have_child(end.x, end.y, end.z, dirPlusOne, net_id) == true)
                        q.push(coord_3d_map[end.x][end.y][end.z]);	// enqueue
                } else {
                    end.x -= cube_dir[dirPlusOne][0];
                    end.y -= cube_dir[dirPlusOne][1];
                    end.z -= cube_dir[dirPlusOne][2];
                    break;
                }
            }
            if (i >= 2 || j >= 2)	// have edge
                    {

                start.x = start.x * rr_map->get_tileWidth() + xDetailShift;
                start.y = start.y * rr_map->get_tileHeight() + yDetailShift;
                ++start.z;
                end.x = end.x * rr_map->get_tileWidth() + xDetailShift;
                end.y = end.y * rr_map->get_tileHeight() + yDetailShift;
                ++end.z;
                printf("(%d,%d,%d)-(%d,%d,%d)\n", start.x, start.y, start.z, end.x, end.y, end.z);
            }
        }
        q.pop();
    }
    // the end of a net of output file
    printf("!\n");
}

void Layer_assignment::klat(int net_id) {
    const PinptrList* pin_list = &rr_map->get_nPin(net_id);
    Coordinate_2d *start;

    start = &coor_array[(*pin_list)[0]->get_tileX()][(*pin_list)[0]->get_tileY()];
    global_net_id = net_id;	// LAZY global variable
    global_pin_num = rr_map->get_netPinNumber(net_id);
    global_max_layer = preprocess(net_id);
    // fina a pin as starting point
    // klat start with a pin
    DP(start->x, start->y, 0);
#ifndef FROM2D
    if (temp_global_pin_cost < klat_map[start->x][start->y][0].val)
    if (temp_global_xy_cost <= after_xy_cost)
    printf("id %d, ori xy %d, ori z %d, curr xy %d, curr z %d\n", net_id, temp_global_xy_cost, temp_global_pin_cost, after_xy_cost, klat_map[start->x][start->y][0].val);
#endif
#ifdef POSTPROCESS
    net_info[net_id].z = klat_map[start->x][start->y][0].via_cost;
    global_increase_vo = klat_map[start->x][start->y][0].val;
#endif
    global_pin_cost += klat_map[start->x][start->y][0].val;
    update_path_for_klat(start);
}

int Layer_assignment::count_via_overflow_for_a_segment(int x, int y, int start, int end) {
    int sum = 0, temp;

    if (start > end) {
        temp = start;
        start = end;
        end = temp;
    }
    while (start < end) {
        if (viadensity_map[x][y][start].cur >= viadensity_map[x][y][start].max)
            ++sum;
        ++start;
    }
    return sum;
}

void Layer_assignment::greedy_layer_assignment(int x, int y, int z) {
    int dir, z_min, z_max, k;
    queue<std::reference_wrapper<Coordinate_3d>> q;

    int k_via_overflow, z_via_overflow = 0;

    q.push(coord_3d_map[x][y][z]);
    while (!q.empty()) {
        Coordinate_3d& temp = q.front();
        if (path_map[temp.x][temp.y].val == 2)	// a pin
            z_min = 0;
        else
            z_min = temp.z;
        z_max = temp.z;
        for (dir = 0; dir < 4; ++dir)
            if (path_map[temp.x][temp.y].edge[dir] == 1)	// check legal
                    {
                x = temp.x + plane_dir[dir][0];
                y = temp.y + plane_dir[dir][1];
#ifdef FOLLOW_PREFER
                for(k = 0; k < max_zz; ++k)
                if ((dir == 0 || dir == 1) && prefer_direction[k][1] == 1)
                break;
                else if ((dir == 2 || dir == 3) && prefer_direction[k][0] == 1)
                break;
#else
                k = 0;
#endif
                k_via_overflow = count_via_overflow_for_a_segment(temp.x, temp.y, k, z_max);
                for (z = k + 1; z < max_zz; ++z)
#ifdef FOLLOW_PREFER
                    if (((dir == 0 || dir == 1) && prefer_direction[z][1] == 1) || ((dir == 2 || dir == 3) && prefer_direction[z][0] == 1))
                    {
#endif
                    if (cur_map_3d[temp.x][temp.y][z].edge_list[dir]->cur_cap < cur_map_3d[temp.x][temp.y][z].edge_list[dir]->max_cap) {
#ifdef VIA_DENSITY
                        if (cur_map_3d[temp.x][temp.y][k].edge_list[dir]->cur_cap < cur_map_3d[temp.x][temp.y][k].edge_list[dir]->max_cap) {
                            z_via_overflow = count_via_overflow_for_a_segment(temp.x, temp.y, MIN(z, z_min), MAX(z, z_max));
                            if (z_via_overflow < k_via_overflow) {
                                k = z;
                                k_via_overflow = z_via_overflow;
                            } else if (z_via_overflow == k_via_overflow) {
                                if (abs(z - temp.z) < abs(k - temp.z)) {
                                    k = z;
                                    k_via_overflow = z_via_overflow;
                                }
                            }
                        } else if (cur_map_3d[temp.x][temp.y][k].edge_list[dir]->cur_cap >= cur_map_3d[temp.x][temp.y][k].edge_list[dir]->max_cap) {
                            k = z;
                            k_via_overflow = z_via_overflow;
                        }
#else
                        if ((cur_map_3d[temp.x][temp.y][k].edge_list[dir]->cur_cap < cur_map_3d[temp.x][temp.y][k].edge_list[dir]->max_cap && abs(z - temp.z) < abs(k - temp.z)) || (cur_map_3d[temp.x][temp.y][k].edge_list[dir]->cur_cap >= cur_map_3d[temp.x][temp.y][k].edge_list[dir]->max_cap))
                        k = z;
#endif
                    } else {
#ifdef VIA_DENSITY
                        if (cur_map_3d[temp.x][temp.y][z].edge_list[dir]->cur_cap - cur_map_3d[temp.x][temp.y][z].edge_list[dir]->max_cap
                                < cur_map_3d[temp.x][temp.y][k].edge_list[dir]->cur_cap - cur_map_3d[temp.x][temp.y][k].edge_list[dir]->max_cap)
                            k = z;
                        else if (cur_map_3d[temp.x][temp.y][z].edge_list[dir]->cur_cap - cur_map_3d[temp.x][temp.y][z].edge_list[dir]->max_cap
                                == cur_map_3d[temp.x][temp.y][k].edge_list[dir]->cur_cap - cur_map_3d[temp.x][temp.y][k].edge_list[dir]->max_cap) {
                            z_via_overflow = count_via_overflow_for_a_segment(temp.x, temp.y, MIN(z, z_min), MAX(z, z_max));
                            if (z_via_overflow < k_via_overflow) {
                                k = z;
                                k_via_overflow = z_via_overflow;
                            } else if (z_via_overflow == k_via_overflow) {
                                if (abs(z - temp.z) < abs(k - temp.z)) {
                                    k = z;
                                    k_via_overflow = z_via_overflow;
                                }
                            }
                        }
#else
                        if ((cur_map_3d[temp.x][temp.y][z].edge_list[dir]->cur_cap - cur_map_3d[temp.x][temp.y][z].edge_list[dir]->max_cap < cur_map_3d[temp.x][temp.y][k].edge_list[dir]->cur_cap - cur_map_3d[temp.x][temp.y][k].edge_list[dir]->max_cap) || (cur_map_3d[temp.x][temp.y][z].edge_list[dir]->cur_cap - cur_map_3d[temp.x][temp.y][z].edge_list[dir]->max_cap == cur_map_3d[temp.x][temp.y][k].edge_list[dir]->cur_cap - cur_map_3d[temp.x][temp.y][k].edge_list[dir]->max_cap && abs(z - temp.z) < abs(k - temp.z)))
                        k = z;
#endif
                    }
#ifdef FOLLOW_PREFER
            }
#endif
                if (k < z_min)
                    z_min = k;
                if (k > z_max)
                    z_max = k;
                update_cur_map_for_klat_xy(k, &coor_array[temp.x][temp.y], &coor_array[x][y], global_net_id);
                q.push(coord_3d_map[x][y][k]);
            }
        update_cur_map_for_klat_z(z_min, z_max, &coor_array[temp.x][temp.y], global_net_id);
        path_map[temp.x][temp.y].val = 0;
        q.pop();
    }
}

void Layer_assignment::greedy(int net_id) {
    const PinptrList* pin_list = &rr_map->get_nPin(net_id);
    Coordinate_2d *start;

    start = &coor_array[(*pin_list)[0]->get_tileX()][(*pin_list)[0]->get_tileY()];
    global_net_id = net_id; // LAZY global variable
    global_pin_num = rr_map->get_netPinNumber(net_id);
    preprocess(net_id);
    greedy_layer_assignment(start->x, start->y, 0);
}

bool Layer_assignment::comp_temp_net_order(int p, int q) {

    return average_order[q].average < average_order[p].average || //
            (!(average_order[p].average < average_order[q].average) && //
                    rr_map->get_netPinNumber(p) < rr_map->get_netPinNumber(q));
}

int Layer_assignment::backtrace(int n) {
    if (group_set[n].pi != n) {
        group_set[n].pi = backtrace(group_set[n].pi);
        return group_set[n].pi;
    }
    return group_set[n].pi;
}

void Layer_assignment::find_group(int max) {
    int i, j, k;
    LRoutedNetTable::iterator iter, iter2;
    int a, b, pre_solve_counter = 0, temp_cap;
    std::deque<bool> pre_solve(max);
    int max_layer;

    group_set = std::vector<UNION_NODE>(max);

// intitial for is_used_for_BFS
    for (i = 0; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
            is_used_for_BFS[i][j] = -1;
// initial for average_order
    average_order = std::vector<AVERAGE_NODE>(max);
    for (i = 0; i < max; ++i) {
        group_set[i].pi = i;
        group_set[i].sx = group_set[i].sy = 1000000;
        group_set[i].bx = group_set[i].by = -1;
        group_set[i].num = 1;
        pre_solve[i] = true;
        average_order[i].id = i;
        average_order[i].val = 0;
        average_order[i].times = 0;
        average_order[i].vo_times = 0;
    }
    for (i = 1; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j) {
            temp_cap = (congestionMap2d->edge(i, j, DIR_WEST).used_net.size() << 1);
            for (k = 0; k < max_zz && temp_cap > 0; ++k)
                if (cur_map_3d[i][j][k].edge_list[LEFT]->max_cap > 0)
                    temp_cap -= cur_map_3d[i][j][k].edge_list[LEFT]->max_cap;
            if (k == max_zz)
                max_layer = max_zz - 1;
            else
                max_layer = k;
            if (congestionMap2d->edge(i, j, DIR_WEST).isOverflow() == false) {
                if (congestionMap2d->edge(i, j, DIR_WEST).used_net.size() > 0) {
                    for (k = 0; k < max_zz && cur_map_3d[i][j][k].edge_list[LEFT]->max_cap == 0; ++k)
                        ;
                    if (k < max_zz) {
                        if (cur_map_3d[i][j][k].edge_list[LEFT]->max_cap < (int) (congestionMap2d->edge(i, j, DIR_WEST).used_net.size() << 1))
                            for (RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_WEST).used_net.begin(); iter != congestionMap2d->edge(i, j, DIR_WEST).used_net.end(); ++iter) {
                                pre_solve[iter->first] = false;
                            }
                    } else
                        for (RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_WEST).used_net.begin(); iter != congestionMap2d->edge(i, j, DIR_WEST).used_net.end(); ++iter) {
                            pre_solve[iter->first] = false;
                        }
                }
            } else {
                for (RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_WEST).used_net.begin(); iter != congestionMap2d->edge(i, j, DIR_WEST).used_net.end(); ++iter) {
                    pre_solve[iter->first] = false;
                }
            }
            for (RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_WEST).used_net.begin(); iter != congestionMap2d->edge(i, j, DIR_WEST).used_net.end(); ++iter) {
                if (i - 1 < group_set[iter->first].sx)
                    group_set[iter->first].sx = i - 1;
                if (i > group_set[iter->first].bx)
                    group_set[iter->first].bx = i;
                if (j < group_set[iter->first].sy)
                    group_set[iter->first].sy = j;
                if (j > group_set[iter->first].by)
                    group_set[iter->first].by = j;
                average_order[iter->first].val += max_layer;
                ++average_order[iter->first].times;
            }
        }
    for (i = 0; i < max_xx; ++i)
        for (j = 1; j < max_yy; ++j) {
            temp_cap = (congestionMap2d->edge(i, j, DIR_SOUTH).used_net.size() << 1);
            for (k = 0; k < max_zz && temp_cap > 0; ++k)
                if (cur_map_3d[i][j][k].edge_list[BACK]->max_cap > 0)
                    temp_cap -= cur_map_3d[i][j][k].edge_list[BACK]->max_cap;
            if (k == max_zz)
                max_layer = max_zz - 1;
            else
                max_layer = k;
            if (congestionMap2d->edge(i, j, DIR_SOUTH).isOverflow() == false) {
                if (congestionMap2d->edge(i, j, DIR_SOUTH).used_net.size() > 0) {
                    for (k = 0; k < max_zz && cur_map_3d[i][j][k].edge_list[BACK]->max_cap == 0; ++k)
                        ;
                    if (k < max_zz) {
                        if (cur_map_3d[i][j][k].edge_list[BACK]->max_cap < (int) (congestionMap2d->edge(i, j, DIR_SOUTH).used_net.size() << 1))
                            for (RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_SOUTH).used_net.begin(); iter != congestionMap2d->edge(i, j, DIR_SOUTH).used_net.end(); ++iter) {
                                pre_solve[iter->first] = false;
                            }
                    } else
                        for (RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_SOUTH).used_net.begin(); iter != congestionMap2d->edge(i, j, DIR_SOUTH).used_net.end(); ++iter) {
                            pre_solve[iter->first] = false;
                        }
                }
            } else {
                for (RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_SOUTH).used_net.begin(); iter != congestionMap2d->edge(i, j, DIR_SOUTH).used_net.end(); ++iter) {
                    pre_solve[iter->first] = false;
                }
            }
            for (RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_SOUTH).used_net.begin(); iter != congestionMap2d->edge(i, j, DIR_SOUTH).used_net.end(); ++iter) {
                if (i < group_set[iter->first].sx)
                    group_set[iter->first].sx = i;
                if (i > group_set[iter->first].bx)
                    group_set[iter->first].bx = i;
                if (j - 1 < group_set[iter->first].sy)
                    group_set[iter->first].sy = j - 1;
                if (j > group_set[iter->first].by)
                    group_set[iter->first].by = j;
                average_order[iter->first].val += max_layer;
                ++average_order[iter->first].times;
            }
        }
    for (i = 0; i < max; ++i)
        if (pre_solve[i] == true)
            group_set[i].pi = -1;
    for (i = 1; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j) {
            for (k = 0; k < max_zz && cur_map_3d[i][j][k].edge_list[LEFT]->max_cap == 0; ++k) {
            }
            if ((int) (congestionMap2d->edge(i, j, DIR_WEST).used_net.size() << 1) > cur_map_3d[i][j][k].edge_list[LEFT]->max_cap) {
                if (congestionMap2d->edge(i, j, DIR_WEST).used_net.size() > 1) {
                    RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_WEST).used_net.begin();
                    a = backtrace(iter->first);
                    for (iter++; iter != congestionMap2d->edge(i, j, DIR_WEST).used_net.end(); ++iter) {
                        b = backtrace(iter->first);
                        if (a != b) {
                            group_set[b].pi = a;
                            group_set[a].num += group_set[b].num;
                        }
                    }
                }
            }
        }
    for (i = 0; i < max_xx; ++i)
        for (j = 1; j < max_yy; ++j) {
            for (k = 0; k < max_zz && cur_map_3d[i][j][k].edge_list[BACK]->max_cap == 0; ++k) {
            }
            if ((int) (congestionMap2d->edge(i, j, DIR_SOUTH).used_net.size() << 1) > cur_map_3d[i][j][k].edge_list[BACK]->max_cap) {
                if (congestionMap2d->edge(i, j, DIR_SOUTH).used_net.size() > 1) {
                    RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_SOUTH).used_net.begin();
                    a = backtrace(iter->first);
                    for (iter++; iter != congestionMap2d->edge(i, j, DIR_SOUTH).used_net.end(); ++iter) {
                        b = backtrace(iter->first);
                        if (a != b) {
                            group_set[b].pi = a;
                            group_set[a].num += group_set[b].num;
                        }
                    }
                }
            }
        }
    for (i = 0; i < max; ++i) {
        if (temp_buf[2] == '0' || temp_buf[2] == '1')	// normal and random
#ifdef VIA_DENSITY
            average_order[i].average = (double) (rr_map->get_netPinNumber(i)) / (average_order[i].times + average_order[i].bends);
#else
        average_order[i].average = (double)(rr_map->get_netPinNumber(i)) / (average_order[i].times);
#endif
        else if (temp_buf[2] == '2')	// length
            average_order[i].average = (1.0 / (average_order[i].times));
        else if (temp_buf[2] == '3')	// pinnum
            average_order[i].average = (double) (rr_map->get_netPinNumber(i));
        else
            // avgdensity
            average_order[i].average = (double) (average_order[i].val) / (average_order[i].times);
        if (pre_solve[i] == true)
            pre_solve_counter++;
    }
}

void Layer_assignment::initial_BFS_color_map() {
    int i, j, k;

    for (i = 0; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
            for (k = 0; k < max_zz; ++k)
                BFS_color_map[i][j][k] = -1;
}

void Layer_assignment::malloc_BFS_color_map() {
    int i, j;

    BFS_color_map = std::vector<std::vector<std::vector<int>>>(max_xx);
    for (i = 0; i < max_xx; ++i) {
        BFS_color_map[i] = std::vector<std::vector<int>>(max_yy);
        for (j = 0; j < max_yy; ++j)
            BFS_color_map[i][j] = std::vector<int>(max_zz);
    }
}

void Layer_assignment::calculate_wirelength() {
    int i, j, k, xy = 0, z = 0;
    LRoutedNetTable::iterator iter;

    for (k = 0; k < max_zz; ++k) {
        for (i = 1; i < max_xx; ++i)
            for (j = 0; j < max_yy; ++j)
                if (cur_map_3d[i][j][k].edge_list[LEFT]->used_net.size()) {
                    xy += cur_map_3d[i][j][k].edge_list[LEFT]->used_net.size();
                }
        for (i = 0; i < max_xx; ++i)
            for (j = 1; j < max_yy; ++j)
                if (cur_map_3d[i][j][k].edge_list[BACK]->used_net.size()) {
                    xy += cur_map_3d[i][j][k].edge_list[BACK]->used_net.size();
                }
    }
    for (i = 0; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
            for (k = 1; k < max_zz; ++k)
                z += (via_cost * cur_map_3d[i][j][k].edge_list[DOWN]->used_net.size());
#ifdef CHANGE_VIA_DENSITY
    printf("%.2lf\t%.2lf\t", (double)(xy + z) / 1.0e5, (double)z / 1.0e5);
#else
    printf("total wirelength = %d + %d = %d\n", xy, z, xy + z);
#endif
}

void Layer_assignment::erase_cur_map_3d() {
    int i, j, k;

    for (i = 1; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
            for (k = 0; k < max_zz; ++k) {
                cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap = 0;
                cur_map_3d[i][j][k].edge_list[LEFT]->used_net.clear();
            }
    for (i = 0; i < max_xx; ++i)
        for (j = 1; j < max_yy; ++j)
            for (k = 0; k < max_zz; ++k) {
                cur_map_3d[i][j][k].edge_list[BACK]->cur_cap = 0;
                cur_map_3d[i][j][k].edge_list[BACK]->used_net.clear();
            }
    for (k = 1; k < max_zz; ++k)
        for (i = 0; i < max_xx; ++i)
            for (j = 0; j < max_yy; ++j)
                cur_map_3d[i][j][k].edge_list[DOWN]->used_net.clear();
}

void Layer_assignment::multiply_viadensity_map_by_times(double times) {
    int i, j, k;

    for (k = 1; k < max_zz; ++k) {
        double temp =
                (cur_map_3d[1][0][k - 1].edge_list[LEFT]->max_cap > 0) ?
                        (cur_map_3d[1][0][k - 1].edge_list[LEFT]->max_cap * cur_map_3d[0][1][k].edge_list[BACK]->max_cap) :
                        (cur_map_3d[1][0][k].edge_list[LEFT]->max_cap * cur_map_3d[0][1][k - 1].edge_list[BACK]->max_cap);
        temp = times * temp;
        temp /= 4.0;
        for (i = 0; i < max_xx; ++i)
            for (j = 0; j < max_yy; ++j) {
                viadensity_map[i][j][k - 1].max = (int) temp;
            }
    }
}

void Layer_assignment::sort_net_order() {
    int i, max;
    LRoutedNetTable count_data;
    LRoutedNetTable::iterator iter;
    LRoutedNetTable two_pin_data;
//vector<Pin*>* pin_list;
    clock_t start, finish;
#ifdef CHANGE_VIA_DENSITY
    double times;
#endif

    max = rr_map->get_netNumber();
// re-disturibte net
    multi_pin_net = std::vector<MULTIPIN_NET_NODE>(max);
    find_group(max);
    initial_overflow_map();
    std::vector<int> temp_net_order(max);
    for (i = 0; i < max; ++i) {
        temp_net_order[i] = i;
    }
    std::sort(temp_net_order.begin(), temp_net_order.end(), [](int a,int b) {return comp_temp_net_order(a,b);});

    start = clock();
#ifdef CHANGE_VIA_DENSITY
    for(times = 0.8; times <= 1.2; times += 0.2)
    {
        initial_overflow_map();
        multiply_viadensity_map_by_times(times);
        for(k = 1; k < max_zz; ++k)
        for(i = 0; i < max_xx; ++i)
        for(j = 0; j < max_yy; ++j)
        viadensity_map[i][j][k-1].cur = 0;
#endif
#ifdef POSTPROCESS
    for(i = 0; i < max; ++i)
    net_info[i].xy = 0;
#endif
    for (i = 0; i < max; ++i) {
        if (temp_buf[3] == '0') // SOLA+APEC
                {
            if (temp_buf[2] == '1')	// random
                klat(i);
            else
                klat(temp_net_order[i]);	// others
        } else	// greedy
        {
            if (temp_buf[2] == '1')	// random
                greedy(i);
            else
                greedy(temp_net_order[i]);	// others
        }
    }
#ifdef CHANGE_VIA_DENSITY
    print_via_overflow();
    calculate_wirelength();
    erase_cur_map_3d();
}
#endif
    printf("cost = %d\n", global_pin_cost);
    finish = clock();
    printf("time = %lf\n", (double) (finish - start) / CLOCKS_PER_SEC);

}

void Layer_assignment::calculate_cap() {
    int i, j, overflow = 0, max = 0;

    for (i = 1; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j) {
            Edge_2d& edge = construct_2d_tree.congestionMap2d->edge(i, j, DIR_WEST);
            if (edge.isOverflow()) {
                overflow += (edge.overUsage() << 1);
                if (max < edge.overUsage())
                    max = edge.overUsage() << 1;
            }
        }
    for (i = 0; i < max_xx; ++i)
        for (j = 1; j < max_yy; ++j) {
            Edge_2d& edge = construct_2d_tree.congestionMap2d->edge(i, j, DIR_SOUTH);
            if (edge.isOverflow()) {
                overflow += (edge.overUsage() << 1);
                if (max < edge.overUsage())
                    max = edge.overUsage() << 1;
            }
        }
    printf("2D overflow = %d\n", overflow);
    printf("2D max overflow = %d\n", max);
}

void Layer_assignment::generate_all_output() {
    int i, max = construct_2d_tree.rr_map.get_netNumber();

    for (i = 0; i < max; ++i)
        generate_output(i);
}

Layer_assignment::Layer_assignment(const std::string& outputFileNamePtr, Construct_2d_tree& construct_2d_tree) :
        construct_2d_tree { construct_2d_tree }, cur_map_3d { construct_2d_tree.cur_map_3d } {

    string outputFileName { outputFileNamePtr };
    //int i;
    via_cost = 1;

    max_xx = rr_map->get_gridx();
    max_yy = rr_map->get_gridy();
    max_zz = rr_map->get_layerNumber();
    malloc_space();
#ifdef READ_OUTPUT
    printf("reading output...\n");
    //read_output();
    printf("reading output complete\n");
#endif
    calculate_cap();
    initial_3D_coordinate_map();
    find_overflow_max();
    puts("Layerassignment processing...");
#ifdef FROM2D
#ifdef POSTPROCESS
    net_info = std::vector<NET_INFO_NODE>(rr_map->get_netNumber());
#endif
    sort_net_order();
    print_max_overflow();
#ifdef POSTPROCESS
    //refinement();
#endif
#endif
    puts("Layerassignment complete.");
#ifdef VIA_DENSITY
    //print_via_overflow();
#endif

    calculate_wirelength();
#ifdef PRINT_OVERFLOW
    print_max_overflow();
#endif
#ifdef ALLOUTPUT
    printf("Outputing result file to %s\n", outputFileName.c_str());
    malloc_BFS_color_map();
    initial_BFS_color_map();

    int stdout_fd = dup(1);
    FILE* outputFile = freopen(outputFileName.c_str(), "w", stdout);
    generate_all_output();
    fclose(outputFile);

    stdout = fdopen(stdout_fd, "w");

#endif
#ifdef CHECK_PATH
    check_path();
#endif
}
