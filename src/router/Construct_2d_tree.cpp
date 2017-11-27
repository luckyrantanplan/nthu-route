#include "Construct_2d_tree.h"
#include "parameter.h"
#include "Route_2pinnets.h"

#include "../flute/flute4nthuroute.h"
#include "../grdb/RoutingComponent.h"
#include "../util/traversemap.h"
#include "../misc/geometry.h"
#include "Post_processing.h"
#include <cmath>
#include <climits>
#include <algorithm>
#include "MM_mazeroute.h"
#include "../grdb/RoutingRegion.h"

using namespace std;
using namespace Jm;

/***********************
 * Global Variable Begin
 * ********************/

Edge_2d::Edge_2d() :
        cur_cap(0.), max_cap(0.), history(1), used_net(128) {

}

Edge_3d::Edge_3d() :
        max_cap(0), cur_cap(0), used_net(5) {

}

Edge_3d_ptr Create_Edge_3d() {
    return new Edge_3d;
}

/*==================DEBUG FUNCTION================================*/
//Obtain the max. overflow and total overflowed value of edges of every gCell
int Construct_2d_tree::cal_max_overflow() {
    int max_2d_of = 0;       //max. overflow (2D)
    int dif_curmax = 0;

    //obtain the max. overflow and total overflowed value of RIGHT edge of every gCell
    for (int i = rr_map.get_gridx() - 2; i >= 0; --i) {
        for (int j = rr_map.get_gridy() - 1; j >= 0; --j) {
            if (congestionMap2d->edge(i, j, DIR_EAST).isOverflow())	//overflow occur
            {
                max_2d_of = max(max_2d_of, congestionMap2d->edge(i, j, DIR_EAST).overUsage());
                dif_curmax += congestionMap2d->edge(i, j, DIR_EAST).overUsage();
            }
        }
    }

    //obtain the max. overflow and total overflowed value of FRONT edge of every gCell
    for (int i = rr_map.get_gridx() - 1; i >= 0; --i) {
        for (int j = rr_map.get_gridy() - 2; j >= 0; --j) {
            if (congestionMap2d->edge(i, j, DIR_NORTH).isOverflow())	//overflow occur
            {
                max_2d_of = max(max_2d_of, congestionMap2d->edge(i, j, DIR_NORTH).overUsage());
                dif_curmax += congestionMap2d->edge(i, j, DIR_NORTH).overUsage();
            }
        }
    }

    printf("\033[32mcal max overflow=%d   cur_cap-max_cap=%d\033[m\n", max_2d_of, dif_curmax);
    return dif_curmax;
}

//Sum all demand value on every edge
//So if demand vlaue = wire length, this function can be used
int Construct_2d_tree::cal_total_wirelength() {
    int total_wl = 0;

    for (int i = rr_map.get_gridx() - 2; i >= 0; --i) {
        for (int j = rr_map.get_gridy() - 1; j >= 0; --j) {
            total_wl += (int) congestionMap2d->edge(i, j, RIGHT).cur_cap;
        }
    }
    for (int i = rr_map.get_gridx() - 1; i >= 0; --i) {
        for (int j = rr_map.get_gridy() - 2; j >= 0; --j) {
            total_wl += (int) congestionMap2d->edge(i, j, FRONT).cur_cap;
        }
    }
    printf("total wirelengh:%d\n", total_wl);
    return total_wl;
}

/*sort bbox in ascending order, then pin_num in descending order*/
bool Construct_2d_tree::comp_net(const Net* a, const Net* b) {
    if (a->get_bboxSize() > b->get_bboxSize()) {
        return true;
    } else if (a->get_bboxSize() < b->get_bboxSize()) {
        return false;
    } else {
        return (a->get_pinNumber() < b->get_pinNumber());
    }
}

bool Construct_2d_tree::comp_2pin_net(Two_pin_element *a, Two_pin_element *b) {
    int a_bbox_size = abs(a->pin1.x - a->pin2.x) + abs(a->pin1.y - a->pin2.y);
    int b_bbox_size = abs(b->pin1.x - b->pin2.x) + abs(b->pin1.y - b->pin2.y);
    return (a_bbox_size < b_bbox_size);
}

bool Construct_2d_tree::comp_2pin_net_from_path(Two_pin_element_2d *a, Two_pin_element_2d *b) {
    int a_bbox_size = abs(a->pin1.x - a->pin2.x) + abs(a->pin1.y - a->pin2.y);
    int b_bbox_size = abs(b->pin1.x - b->pin2.x) + abs(b->pin1.y - b->pin2.y);
    return (a_bbox_size < b_bbox_size);
}

//sort by x,y,pin,steiner
bool Construct_2d_tree::comp_vertex_fl(Vertex_flute_ptr a, Vertex_flute_ptr b) {
    if (a->x < b->x)
        return true;
    else if (a->x > b->x)
        return false;
    else if (a->y < b->y)
        return true;
    else if (a->y > b->y)
        return false;
    else if (a->type == PIN)
        return true;
    else
        return false;
}

void Construct_2d_tree::setup_flute_order(int *order) {
    for (int i = global_flutetree.number - 1; i >= 0; --i) {
        order[i] = i;
    }
}

/*assign the estimated track# to each edge*/
void Construct_2d_tree::init_2d_map() {
    congestionMap2d = new EdgePlane<Edge_2d>(rr_map.get_gridx(), rr_map.get_gridy(), Edge_2d());

    for (int x = rr_map.get_gridx() - 2; x >= 0; --x) {
        for (int y = rr_map.get_gridy() - 1; y >= 0; --y) {
            for (int layer = rr_map.get_layerNumber() - 1; layer >= 0; --layer) {
#ifdef IBM_CASE		
                //There is no wire spacing, so
                //the edge capacity on congestion map = edge capacity on every layer
                congestionMap2d->edge(x, y, DIR_EAST).max_cap += rr_map.capacity(layer, x, y, x+1, y);
#else
                //Because the wire width = 1 and wire spaceing = 1,
                //the edge capacity on congestion map = edge capacity on every layer /2.
                congestionMap2d->edge(x, y, DIR_EAST).max_cap += (rr_map.capacity(layer, x, y, x + 1, y) / 2);
#endif		
            }
        }
    }

    for (int x = rr_map.get_gridx() - 1; x >= 0; --x) {
        for (int y = rr_map.get_gridy() - 2; y >= 0; --y) {
            for (int layer = rr_map.get_layerNumber() - 1; layer >= 0; --layer) {
#ifdef IBM_CASE		
                //There is no wire spacing, so
                //the edge capacity on congestion map = edge capacity on every layer
                congestionMap2d->edge(x, y, DIR_NORTH).max_cap += rr_map.capacity(layer, x, y, x, y+1);
#else
                //Because the wire width = 1 and wire spaceing = 1,
                //the edge capacity on congestion map = edge capacity on every layer /2.
                congestionMap2d->edge(x, y, DIR_NORTH).max_cap += (rr_map.capacity(layer, x, y, x, y + 1) / 2);
#endif		
            }
        }
    }
}

//Make an coordinate array which contains the (x, y) information
void Construct_2d_tree::allocate_coor_array() {
    int i, j;
    Coordinate_2d *tmp_data;

    coor_array = (Coordinate_2d **) malloc(rr_map.get_gridx() * sizeof(Coordinate_2d *));
    tmp_data = (Coordinate_2d *) malloc(rr_map.get_gridx() * rr_map.get_gridy() * sizeof(Coordinate_2d));
    for (i = 0; i < rr_map.get_gridx(); ++i, tmp_data += rr_map.get_gridy())
        coor_array[i] = tmp_data;

    for (i = 0; i < rr_map.get_gridx(); ++i) {
        for (j = 0; j < rr_map.get_gridy(); ++j) {
            coor_array[i][j].x = i;
            coor_array[i][j].y = j;
        }
    }
}

void Construct_2d_tree::init_3d_map() {
    int i, j, k;
    Vertex_3d **tmp_data, *tmp_data2;
    Edge_3d_ptr newedge;

    /*allocate space for cur_map_3d*/
    cur_map_3d = (Vertex_3d ***) malloc(rr_map.get_gridx() * sizeof(Vertex_3d **));
    tmp_data = (Vertex_3d **) malloc(rr_map.get_gridx() * rr_map.get_gridy() * sizeof(Vertex_3d *));
    for (i = 0; i < rr_map.get_gridx(); ++i, tmp_data += rr_map.get_gridy())
        cur_map_3d[i] = tmp_data;
    tmp_data2 = (Vertex_3d *) malloc(rr_map.get_gridx() * rr_map.get_gridy() * rr_map.get_layerNumber() * sizeof(Vertex_3d));
    for (i = 0; i < rr_map.get_gridx(); ++i)
        for (j = 0; j < rr_map.get_gridy(); ++j, tmp_data2 += rr_map.get_layerNumber())
            cur_map_3d[i][j] = tmp_data2;

    //initialize capacity
    for (i = 0; i < rr_map.get_gridx() - 1; ++i)
        for (j = 0; j < rr_map.get_gridy(); ++j)
            for (k = 0; k < rr_map.get_layerNumber(); ++k) {
                newedge = Create_Edge_3d(); /*allocate space for edge_list without initialization*/
                cur_map_3d[i][j][k].edge_list[RIGHT] = newedge;
                cur_map_3d[i + 1][j][k].edge_list[LEFT] = newedge;
            }
    for (i = 0; i < rr_map.get_gridx(); ++i)
        for (j = 0; j < rr_map.get_gridy() - 1; ++j)
            for (k = 0; k < rr_map.get_layerNumber(); ++k) {
                newedge = Create_Edge_3d(); /*allocate space for edge_list without initialization*/
                cur_map_3d[i][j][k].edge_list[FRONT] = newedge;
                cur_map_3d[i][j + 1][k].edge_list[BACK] = newedge;
            }
    for (i = 0; i < rr_map.get_gridx(); ++i)
        for (j = 0; j < rr_map.get_gridy(); ++j)
            for (k = 0; k < rr_map.get_layerNumber() - 1; ++k) {
                newedge = Create_Edge_3d(); /*allocate space for edge_list without initialization*/
                cur_map_3d[i][j][k].edge_list[UP] = newedge;
                cur_map_3d[i][j][k + 1].edge_list[DOWN] = newedge;
            }
    for (j = 0; j < rr_map.get_gridy(); ++j)
        for (k = 0; k < rr_map.get_layerNumber(); ++k)
            cur_map_3d[0][j][k].edge_list[LEFT] = cur_map_3d[rr_map.get_gridx() - 1][j][k].edge_list[RIGHT] = NULL;
    for (i = 0; i < rr_map.get_gridx(); ++i)
        for (k = 0; k < rr_map.get_layerNumber(); ++k)
            cur_map_3d[i][0][k].edge_list[BACK] = cur_map_3d[i][rr_map.get_gridy() - 1][k].edge_list[FRONT] = NULL;
    for (i = 0; i < rr_map.get_gridx(); ++i)
        for (j = 0; j < rr_map.get_gridy(); ++j)
            cur_map_3d[i][j][0].edge_list[DOWN] = cur_map_3d[i][j][rr_map.get_layerNumber() - 1].edge_list[UP] = NULL;
}

void Construct_2d_tree::init_2pin_list() {
    int i, netnum;

    netnum = rr_map.get_netNumber();
    for (i = 0; i < netnum; ++i) {
        //Two pin nets group by net id. So for fetching the 2nd net's 2-pin net,
        //you can fetch by net_2pin_list[2][i], where i is the id of 2-pin net.
        net_2pin_list.push_back(new Two_pin_list_2d);
        bbox_2pin_list.push_back(new Two_pin_list_2d);
    }
}

void Construct_2d_tree::init_flute() {
    net_flutetree = (Tree *) malloc(rr_map.get_netNumber() * sizeof(Tree));
}

void Construct_2d_tree::free_memory_con2d() {
    for (vector<Two_pin_list_2d*>::iterator it = bbox_2pin_list.begin(); it != bbox_2pin_list.end(); ++it)
        delete (*it);
    bbox_2pin_list.clear();
}

void Construct_2d_tree::bbox_route(Two_pin_list_2d *list, const double value) {
    int i, x1, y1, x2, y2;
    double u_value;

    if (value > 0)
        u_value = 1;
    else
        u_value = -1;

    for (vector<Two_pin_element_2d*>::iterator it = list->begin(); it != list->end(); ++it) {
        if ((*it)->pin1.x > (*it)->pin2.x) {
            swap((*it)->pin1.x, (*it)->pin2.x);
        }
        if ((*it)->pin1.y > (*it)->pin2.y) {
            swap((*it)->pin1.y, (*it)->pin2.y);
        }

        x1 = (*it)->pin1.x;
        y1 = (*it)->pin1.y;
        x2 = (*it)->pin2.x;
        y2 = (*it)->pin2.y;
#ifdef DEBUG_BBOX
        printf("(%d %d) (%d %d)\n",x1,y1,x2,y2);
#endif
        if (x1 == x2)	    //vertical edge
                {
            for (i = y1; i < y2; ++i) {
                bboxRouteStateMap->color(x1, i, DIR_NORTH) = 1;
            }
        } else if (y1 == y2)	//horizontal edge
                {
            for (i = x1; i < x2; ++i) {
                bboxRouteStateMap->color(i, y1, DIR_EAST) = 1;
            }
        } else	            //box (L-shape routing need
        {
            for (i = y1; i < y2; ++i) {
                bboxRouteStateMap->color(x1, i, DIR_NORTH) = 1;
                bboxRouteStateMap->color(x2, i, DIR_NORTH) = 1;
            }
            for (i = x1; i < x2; ++i) {
                bboxRouteStateMap->color(i, y1, DIR_EAST) = 1;
                bboxRouteStateMap->color(i, y2, DIR_EAST) = 1;
            }
        }
    }

    //check the flag of edges, if it is set to 1, then add demand on it.
    for (vector<Two_pin_element_2d*>::iterator it = list->begin(); it != list->end(); ++it) {
        //x1, y1 is the smaller set
        //NOTE: they have been sorted in the last for loop
        x1 = (*it)->pin1.x;
        y1 = (*it)->pin1.y;
        x2 = (*it)->pin2.x;
        y2 = (*it)->pin2.y;
        if (x1 == x2)	    //vertical edge
                {
            for (i = y1; i < y2; ++i)
                if (bboxRouteStateMap->color(x1, i, DIR_NORTH) == 1) {
                    congestionMap2d->edge(x1, i, DIR_NORTH).cur_cap += u_value;
                    bboxRouteStateMap->color(x1, i, DIR_NORTH) = 0;
                }
        } else if (y1 == y2)	//horizontal edge
                {
            for (i = x1; i < x2; ++i)
                if (bboxRouteStateMap->color(i, y1, DIR_EAST) == 1) {
                    congestionMap2d->edge(i, y1, DIR_EAST).cur_cap += u_value;
                    bboxRouteStateMap->color(i, y1, DIR_EAST) = 0;
                }
        } else	            //box
        {
            for (i = y1; i < y2; ++i) {
                if (bboxRouteStateMap->color(x1, i, DIR_NORTH) == 1) {
                    congestionMap2d->edge(x1, i, DIR_NORTH).cur_cap += value;
                    bboxRouteStateMap->color(x1, i, DIR_NORTH) = 0;
                }
                if (bboxRouteStateMap->color(x2, i, DIR_NORTH) == 1) {
                    congestionMap2d->edge(x2, i, DIR_NORTH).cur_cap += value;
                    bboxRouteStateMap->color(x2, i, DIR_NORTH) = 0;
                }
            }
            for (i = x1; i < x2; ++i) {
                if (bboxRouteStateMap->color(i, y1, DIR_EAST) == 1) {
                    congestionMap2d->edge(i, y1, DIR_EAST).cur_cap += value;
                    bboxRouteStateMap->color(i, y1, DIR_EAST) = 0;
                }
                if (bboxRouteStateMap->color(i, y2, DIR_EAST) == 1) {
                    congestionMap2d->edge(i, y2, DIR_EAST).cur_cap += value;
                    bboxRouteStateMap->color(i, y2, DIR_EAST) = 0;
                }
            }
        }
    }
#ifdef DEBUG_BBOX
    print_cap("cur");
#endif
}

void Construct_2d_tree::insert_all_two_pin_list(Two_pin_element_2d *mn_path_2d) {
    Two_pin_element *mn_path;
    mn_path = new Two_pin_element();
    mn_path->pin1.x = mn_path_2d->pin1.x;
    mn_path->pin1.y = mn_path_2d->pin1.y;
    mn_path->pin2.x = mn_path_2d->pin2.x;
    mn_path->pin2.y = mn_path_2d->pin2.y;
    mn_path->pin1.z = mn_path->pin2.z = 0;
    mn_path->net_id = mn_path_2d->net_id;
    all_two_pin_list.push_back(mn_path);
}

bool Construct_2d_tree::smaller_than_lower_bound(double total_cost, int distance, int via_num, double bound_cost, int bound_distance, int bound_via_num) {
    if ((total_cost - bound_cost) < neg_error_bound)
        return true;
    else if ((total_cost - bound_cost) > error_bound)
        return false;
    else {
        if (distance < bound_distance)
            return true;
        else if (distance > bound_distance)
            return false;
        else {
            return (via_num < bound_via_num);
        }
    }
}

/* *NOTICE*
 * You can create many different cost function for difference case easily, 
 * just reassign function ponter pre_evaluate_congestion_cost_fp to your 
 * function in *route/route.cpp* .                                        */

void (*pre_evaluate_congestion_cost_fp)(int i, int j, int dir);
void Construct_2d_tree::pre_evaluate_congestion_cost_all(int i, int j, int dir) {
    static const int inc = 1;
    double cong;

    DirectionType dirType = static_cast<DirectionType>(Jr2JmDirArray[dir]);

    if (used_cost_flag == HISTORY_COST) {
        cong = (congestionMap2d->edge(i, j, dirType).cur_cap + inc)
                / (congestionMap2d->edge(i, j, dirType).max_cap * (1.0 - ((congestionMap2d->edge(i, j, dirType).history - 1) / (cur_iter * (1.5 + 3 * factor)))));
        cache->edge(i, j, dirType).cost = WL_Cost + (congestionMap2d->edge(i, j, dirType).history) * pow(cong, exponent);
    } else {
        if (congestionMap2d->edge(i, j, dirType).isFull())
            cache->edge(i, j, dirType).cost = 1.0;
        else
            cache->edge(i, j, dirType).cost = 0.0;
    }
}

void Construct_2d_tree::pre_evaluate_congestion_cost() {
    for (int i = rr_map.get_gridx() - 1; i >= 0; --i) {
        for (int j = rr_map.get_gridy() - 2; j >= 0; --j) {
            pre_evaluate_congestion_cost_fp(i, j, FRONT); // Function Pointer to Cost function
            if (congestionMap2d->edge(i, j, DIR_NORTH).isOverflow()) {
                ++congestionMap2d->edge(i, j, DIR_NORTH).history;
            }
        }
    }
    for (int i = rr_map.get_gridx() - 2; i >= 0; --i) {
        for (int j = rr_map.get_gridy() - 1; j >= 0; --j) {
            pre_evaluate_congestion_cost_fp(i, j, RIGHT);
            if (congestionMap2d->edge(i, j, DIR_EAST).isOverflow()) {
                ++congestionMap2d->edge(i, j, DIR_EAST).history;
            }
        }
    }
}

//get edge cost on a 2D layer
double Construct_2d_tree::get_cost_2d(int i, int j, int dir, int net_id, int *distance) {
    DirectionType dirType = static_cast<DirectionType>(Jr2JmDirArray[dir]);
    //Check if the specified net pass the edge.
    //If it have passed the edge before, then the cost is 0.
    if (congestionMap2d->edge(i, j, dirType).lookupNet(net_id) == false) {
        (*distance) = 1;

        //Used in part II
        if (used_cost_flag == HISTORY_COST) {
            return cache->edge(i, j, dirType).cost;
        }

        //Used in part III: Post processing
        else if (used_cost_flag == MADEOF_COST) {
            return congestionMap2d->edge(i, j, dirType).isFull();
        }

        //Used in part I: Initial routing
        else if (used_cost_flag == FASTROUTE_COST) {
            return 1 + parameter_h / (1 + exp((-1) * parameter_k * (congestionMap2d->edge(i, j, dirType).cur_cap + 1 - congestionMap2d->edge(i, j, dirType).max_cap)));
        }

        return 0;
    } else {
        (*distance) = 0;
        return 0;
    }
}

/*
 Compare two cost and return a pointer to the Monotonici_element which has smaller cost
 */
Monotonic_element* Construct_2d_tree::compare_cost(Monotonic_element* m1, Monotonic_element* m2) {
    if ((m1->total_cost - m2->total_cost) < (neg_error_bound))
        return m1;
    else if ((m1->total_cost - m2->total_cost) > (neg_error_bound))
        return m2;
    else {
        if ((m1->max_cost - m2->max_cost) < (neg_error_bound))
            return m1;
        else if ((m1->max_cost - m2->max_cost) > (neg_error_bound))
            return m2;
        else {
            if (m1->distance < m2->distance)
                return m1;
            else if (m1->distance > m2->distance)
                return m2;
            else {
                if (m1->via_num <= m2->via_num)
                    return m1;
                else
                    return m2;
            }
        }
    }
}

/*
 input: start coor and end coor, and directions of L
 output: record the best L pattern into two_pin_L_path_global, and return the min max congestion value
 */
Monotonic_element Construct_2d_tree::L_pattern_max_cong(int x1, int y1, int x2, int y2, int dir1, int dir2, Two_pin_element_2d* two_pin_L_path, int net_id) {
    int i, j;
    double temp;
    int dir[2], dir_index;
    Monotonic_element max_path;
    int distance;

    dir[0] = dir1;
    dir[1] = dir2;
    i = x1;
    j = y1;
    max_path.max_cost = -1000000;
    max_path.total_cost = 0;
    max_path.net_cost = 0;
    max_path.distance = 0;
    max_path.via_num = 1;
    for (dir_index = 0; dir_index < 2; dir_index++) {
        if (dir[dir_index] == RIGHT)    // search in horizontal direction: RIGHT
                {
            //for loop from the left boundary to the right boundary
            for (i = x1; i < x2; ++i) {
                temp = get_cost_2d(i, j, dir[dir_index], net_id, &distance);
                max_path.total_cost += max(static_cast<double>(0), temp);
                max_path.distance += distance;
                (*two_pin_L_path).path.push_back(&coor_array[i][j]);
                if (temp > max_path.max_cost)
                    max_path.max_cost = temp;
            }
            i = x2;
        } else // search in vertical direction
        {
            for (j = y1; (j < y2 && dir[dir_index] == FRONT) || (j > y2 && dir[dir_index] == BACK);) {
                temp = get_cost_2d(i, j, dir[dir_index], net_id, &distance);
                max_path.total_cost += max(static_cast<double>(0), temp);
                max_path.distance += distance;
                (*two_pin_L_path).path.push_back(&coor_array[i][j]);
                if (temp > max_path.max_cost)
                    max_path.max_cost = temp;
                if (dir[dir_index] == FRONT)
                    ++j;
                else if (dir[dir_index] == BACK)
                    --j;
            }
            j = y2;
        }
    }

    (*two_pin_L_path).path.push_back(&coor_array[x2][y2]);
    return max_path;
}

/*
 input: two coordinates
 output: record the L_pattern in the path, and the path is min max congestioned
 */
void Construct_2d_tree::L_pattern_route(int x1, int y1, int x2, int y2, Two_pin_element_2d* two_pin_L_path, int net_id) {
    Two_pin_element_2d path1, path2;
    Monotonic_element max_cong_path1, max_cong_path2;

    if (x1 > x2) {
        swap(x1, x2);
        swap(y1, y2);
    }

    if (x1 < x2 && y1 < y2)					//two points are left_back and right_front
            {
        //FRONT and RIGHT L pattern from (x1 y1)
        max_cong_path1 = L_pattern_max_cong(x1, y1, x2, y2, FRONT, RIGHT, &path1, net_id);
        //RIGTH and FRONT L pattern from (x1,y1)
        max_cong_path2 = L_pattern_max_cong(x1, y1, x2, y2, RIGHT, FRONT, &path2, net_id);
        if ((&max_cong_path1) == (compare_cost(&max_cong_path1, &max_cong_path2)))
            (*two_pin_L_path) = path1;
        else
            (*two_pin_L_path) = path2;
    } else if (x1 < x2 && y1 > y2)  //two points are left_front and right_back
            {
        //BACK and RIGHT L pattern from (x1,y1)
        max_cong_path1 = L_pattern_max_cong(x1, y1, x2, y2, BACK, RIGHT, &path1, net_id);
        //RIGHT and BACK L pattern from (x1,y1)
        max_cong_path2 = L_pattern_max_cong(x1, y1, x2, y2, RIGHT, BACK, &path2, net_id);
        if ((&max_cong_path1) == (compare_cost(&max_cong_path1, &max_cong_path2)))
            (*two_pin_L_path) = path1;
        else
            (*two_pin_L_path) = path2;
    } else // vertical or horizontal line
    {
        if (y1 > y2) {
            swap(y1, y2);
        }
        max_cong_path1 = L_pattern_max_cong(x1, y1, x2, y2, FRONT, RIGHT, two_pin_L_path, net_id);
    }
    (*two_pin_L_path).pin1 = *((*two_pin_L_path).path[0]);
    (*two_pin_L_path).pin2 = *((*two_pin_L_path).path.back());
    (*two_pin_L_path).net_id = net_id;
}

/*allocate space for monotonic pattern routing*/
void Construct_2d_tree::allocate_monotonic() {
    Monotonic_element *tmp_data;
    int *tmp_data2;

    /*allocate space for cong_monotonic*/
    cong_monotonic = (Monotonic_element **) malloc(rr_map.get_gridx() * sizeof(Monotonic_element *));
    tmp_data = (Monotonic_element *) malloc(rr_map.get_gridx() * rr_map.get_gridy() * sizeof(Monotonic_element));
    for (int i = 0; i < rr_map.get_gridx(); ++i, tmp_data += rr_map.get_gridy())
        cong_monotonic[i] = tmp_data;

    /*allocate space for parent_monotonic*/
    parent_monotonic = (int **) malloc(rr_map.get_gridx() * sizeof(int *));
    tmp_data2 = (int *) malloc(rr_map.get_gridx() * rr_map.get_gridy() * sizeof(int));
    for (int i = 0; i < rr_map.get_gridx(); ++i, tmp_data2 += rr_map.get_gridy())
        parent_monotonic[i] = tmp_data2;
}

void Construct_2d_tree::compare_two_direction_congestion(int i, int j, int dir1, int pre_i, int dir2, int pre_j, int net_id, double bound_cost, int bound_distance, int bound_via_num,
        bool bound_flag) {
    Monotonic_element left_element, vertical_element, *choose_element;
    double cost;
    int distance = 1;
    bool left_flag, right_flag;
    int pre_dir;

    left_flag = right_flag = true;
    if (parent_monotonic[pre_i][j] != -2) {
        cost = get_cost_2d(i, j, dir1, net_id, &distance);
        left_element.max_cost = max(cost, cong_monotonic[pre_i][j].max_cost);
        left_element.total_cost = cong_monotonic[pre_i][j].total_cost + max(static_cast<double>(0), cost);
        left_element.distance = cong_monotonic[pre_i][j].distance + distance;
        pre_dir = parent_monotonic[pre_i][j];
        if (pre_dir < 2) {
            left_element.via_num = cong_monotonic[pre_i][j].via_num + via_cost;
            if (distance != 0) {
                left_element.distance += via_cost;
                if (used_cost_flag == HISTORY_COST)
                    left_element.total_cost += via_cost;
            }
        } else
            left_element.via_num = cong_monotonic[pre_i][j].via_num;
        if (!bound_flag || (bound_flag && smaller_than_lower_bound(left_element.total_cost, left_element.distance, left_element.via_num, bound_cost, bound_distance, bound_via_num))) {
            left_flag = true;
        } else
            left_flag = false;
    } else
        left_flag = false;
    if (parent_monotonic[i][pre_j] != -2) {
        cost = get_cost_2d(i, j, dir2, net_id, &distance);
        vertical_element.max_cost = max(cost, cong_monotonic[i][pre_j].max_cost);
        vertical_element.total_cost = cong_monotonic[i][pre_j].total_cost + max(static_cast<double>(0), cost);
        vertical_element.distance = cong_monotonic[i][pre_j].distance + distance;
        pre_dir = parent_monotonic[i][pre_j];
        if (pre_dir >= 2) {
            vertical_element.via_num = cong_monotonic[i][pre_j].via_num + via_cost;
            if (distance != 0) {
                vertical_element.distance += via_cost;
                if (used_cost_flag == HISTORY_COST)
                    vertical_element.total_cost += via_cost;
            }
        } else
            vertical_element.via_num = cong_monotonic[i][pre_j].via_num;
        if (!bound_flag || (bound_flag && smaller_than_lower_bound(vertical_element.total_cost, vertical_element.distance, vertical_element.via_num, bound_cost, bound_distance, bound_via_num))) {
            right_flag = true;
        } else
            right_flag = false;
    } else
        right_flag = false;

    if ((!left_flag) && (!right_flag)) {
        parent_monotonic[i][j] = -2;
        return;
    } else if (left_flag && right_flag)
        choose_element = compare_cost(&left_element, &vertical_element);
    else if (left_flag)
        choose_element = &left_element;
    else
        choose_element = &vertical_element;

    cong_monotonic[i][j].max_cost = choose_element->max_cost;
    cong_monotonic[i][j].total_cost = choose_element->total_cost;
    cong_monotonic[i][j].net_cost = choose_element->net_cost;
    cong_monotonic[i][j].distance = choose_element->distance;
    cong_monotonic[i][j].via_num = choose_element->via_num;
    if (choose_element == (&left_element))
        parent_monotonic[i][j] = dir1;
    else if (choose_element == (&vertical_element))
        parent_monotonic[i][j] = dir2;
    else {
        puts("compare has problem!!!\n");
        exit(0);
    }
}

void Construct_2d_tree::monotonic_routing_algorithm(int x1, int y1, int x2, int y2, int dir, int net_id, double bound_cost, int bound_distance, int bound_via_num, bool bound_flag) {
    int i, j;
    double cost;
    int distance = 1;

    //initialize cong_monotonic and parent_monotonic
    cong_monotonic[x1][y1].max_cost = -1000000;
    cong_monotonic[x1][y1].total_cost = 0;
    cong_monotonic[x1][y1].distance = 0;
    cong_monotonic[x1][y1].net_cost = 0;
    cong_monotonic[x1][y1].via_num = 0;
    parent_monotonic[x1][y1] = -1;
    //Update the cost of top boundary or bottom boundary, which draw with double line.
    //The source can in left-top corner or left-bottom corner
    for (i = x1 + 1; i <= x2; ++i) {
        if (parent_monotonic[i - 1][y1] != -2) {
            cost = get_cost_2d(i, y1, LEFT, net_id, &distance);
            cong_monotonic[i][y1].max_cost = max(cost, cong_monotonic[i - 1][y1].max_cost);
            cong_monotonic[i][y1].total_cost = cong_monotonic[i - 1][y1].total_cost + max(static_cast<double>(0), cost);
            cong_monotonic[i][y1].distance = cong_monotonic[i - 1][y1].distance + distance;
            cong_monotonic[i][y1].via_num = cong_monotonic[i - 1][y1].via_num;

            if (!bound_flag
                    || (bound_flag
                            && smaller_than_lower_bound(cong_monotonic[i][y1].total_cost, cong_monotonic[i][y1].distance, cong_monotonic[i][y1].via_num, bound_cost, bound_distance, bound_via_num))) {
                parent_monotonic[i][y1] = LEFT;
            } else
                parent_monotonic[i][y1] = -2;
        } else
            parent_monotonic[i][y1] = -2;
    }

    //If source is in the left-top corner
    if (dir == BACK) {
        for (j = y1 + 1; j <= y2; ++j) {
            if (parent_monotonic[x1][j - 1] != -2) {
                cost = get_cost_2d(x1, j, dir, net_id, &distance);
                cong_monotonic[x1][j].max_cost = max(cost, cong_monotonic[x1][j - 1].max_cost);
                cong_monotonic[x1][j].total_cost = cong_monotonic[x1][j - 1].total_cost + max(static_cast<double>(0), cost);
                cong_monotonic[x1][j].distance = cong_monotonic[x1][j - 1].distance + distance;
                cong_monotonic[x1][j].via_num = cong_monotonic[x1][j - 1].via_num;
                if (!bound_flag
                        || (bound_flag
                                && smaller_than_lower_bound(cong_monotonic[x1][j].total_cost, cong_monotonic[x1][j].distance, cong_monotonic[x1][j].via_num, bound_cost, bound_distance, bound_via_num))) {
                    parent_monotonic[x1][j] = dir;
                } else
                    parent_monotonic[x1][j] = dir;
            } else
                parent_monotonic[x1][j] = -2;
        }

        //If source is in the left-bottom corner
    } else if (dir == FRONT) {
        for (j = y1 - 1; j >= y2; --j) {
            if (parent_monotonic[x1][j + 1] != -2) {
                cost = get_cost_2d(x1, j, dir, net_id, &distance);
                cong_monotonic[x1][j].max_cost = max(cost, cong_monotonic[x1][j + 1].max_cost);
                cong_monotonic[x1][j].total_cost = cong_monotonic[x1][j + 1].total_cost + max(static_cast<double>(0), cost);
                cong_monotonic[x1][j].distance = cong_monotonic[x1][j + 1].distance + distance;
                cong_monotonic[x1][j].via_num = cong_monotonic[x1][j + 1].via_num;
                if (!bound_flag
                        || (bound_flag
                                && smaller_than_lower_bound(cong_monotonic[x1][j].total_cost, cong_monotonic[x1][j].distance, cong_monotonic[x1][j].via_num, bound_cost, bound_distance, bound_via_num))) {
                    parent_monotonic[x1][j] = dir;
                } else
                    parent_monotonic[x1][j] = -2;
            } else
                parent_monotonic[x1][j] = -2;
        }
    }

    for (i = x1 + 1; i <= x2; ++i) {
        //If source is in the left-bottom corner
        if (dir == BACK) {
            for (j = y1 + 1; j <= y2; ++j)
                compare_two_direction_congestion(i, j, LEFT, i - 1, dir, j - 1, net_id, bound_cost, bound_distance, bound_via_num, bound_flag);

            //If source is in the left-top corner
        } else {
            for (j = y1 - 1; j >= y2; --j)
                compare_two_direction_congestion(i, j, LEFT, i - 1, dir, j + 1, net_id, bound_cost, bound_distance, bound_via_num, bound_flag);
        }
    }
}

void Construct_2d_tree::traverse_parent_monotonic(int x1, int y1, int x2, int y2, Two_pin_element_2d* two_pin_monotonic_path) {
    int i = x2;
    int j = y2;
    //Sink != Source
    while ((i != x1) || (j != y1)) {
        //Push the path in to a list
        (*two_pin_monotonic_path).path.push_back(&coor_array[i][j]);

        //Update the coordinate of tracing cell
        if (parent_monotonic[i][j] == LEFT)
            --i;
        else if (parent_monotonic[i][j] == FRONT)
            ++j;
        else
            --j;
    }

    //push the source to list
    (*two_pin_monotonic_path).path.push_back(&coor_array[i][j]);
}

//Try to obtain a monotonic routing path without cost over bounding cost
//Return true if there exist one such path
bool Construct_2d_tree::monotonic_pattern_route(int x1, int y1, int x2, int y2, Two_pin_element_2d* two_pin_monotonic_path, int net_id, double bound_cost, int bound_distance, int bound_via_num,
        bool bound_flag) {
    if (x1 > x2) {
        swap(x1, x2);
        swap(y1, y2);
    }
    if (y1 <= y2) //s->t RIGHT and FRONT (source is in the left-bottom corner)
        monotonic_routing_algorithm(x1, y1, x2, y2, BACK, net_id, bound_cost, bound_distance, bound_via_num, bound_flag);
    // use x1,y+1.edge_list[back]
    else if (y1 > y2) //s->t RIGHT and BACK (source is in the left-top corner)
        monotonic_routing_algorithm(x1, y1, x2, y2, FRONT, net_id, bound_cost, bound_distance, bound_via_num, bound_flag);

    //If there is no solution for this 2-pin net, return false
    if (parent_monotonic[x2][y2] == -2)
        return false;

    //travese parent_monotonic to find path in two_pin_monotonic_path
    traverse_parent_monotonic(x1, y1, x2, y2, two_pin_monotonic_path);

    (*two_pin_monotonic_path).pin1 = *((*two_pin_monotonic_path).path[0]);
    (*two_pin_monotonic_path).pin2 = *((*two_pin_monotonic_path).path.back());
    (*two_pin_monotonic_path).net_id = net_id;
    return true;
}

//Add the path of two pin element on to congestion map
//The congestion map record not only which net pass which edge,
//but also the number of a net pass through
void Construct_2d_tree::update_congestion_map_insert_two_pin_net(Two_pin_element_2d *element) {
    int dir;

    NetDirtyBit[element->net_id] = true;

    for (int i = element->path.size() - 2; i >= 0; --i) {
        //get an edge from congestion map - c_map_2d
        dir = get_direction_2d(element->path[i], element->path[i + 1]);

        pair<RoutedNetTable::iterator, bool> insert_result = congestionMap2d->edge(element->path[i]->x, element->path[i]->y, dir).used_net.insert(pair<int, int>(element->net_id, 1));

        if (!insert_result.second)
            ++((insert_result.first)->second);
        else {
            ++(congestionMap2d->edge(element->path[i]->x, element->path[i]->y, dir).cur_cap);

            if (used_cost_flag != FASTROUTE_COST) {
                pre_evaluate_congestion_cost_fp(element->path[i]->x, element->path[i]->y, dir);
            }
        }
    }
}

//Remove a net from an edge.
//If the net pass that edge more than once, this function will only decrease the counter.
void Construct_2d_tree::update_congestion_map_remove_two_pin_net(Two_pin_element_2d *element) {
    int dir;

    NetDirtyBit[element->net_id] = true;

    for (int i = element->path.size() - 2; i >= 0; --i) {
        dir = get_direction_2d(element->path[i], element->path[i + 1]);

        RoutedNetTable::iterator find_result = congestionMap2d->edge(element->path[i]->x, element->path[i]->y, dir).used_net.find(element->net_id);

        --(find_result->second);
        if (find_result->second == 0) {
            congestionMap2d->edge(element->path[i]->x, element->path[i]->y, dir).used_net.erase(element->net_id);
            --(congestionMap2d->edge(element->path[i]->x, element->path[i]->y, dir).cur_cap);
            if (used_cost_flag != FASTROUTE_COST) {
                pre_evaluate_congestion_cost_fp(element->path[i]->x, element->path[i]->y, dir);
            }
        }
    }
}

void Construct_2d_tree::update_congestion_map_remove_multipin_net(Two_pin_list_2d *list) {
    for (vector<Two_pin_element_2d*>::iterator it = list->begin(); it != list->end(); ++it) {
        update_congestion_map_remove_two_pin_net(*it);
    }
}

//generate the congestion map by Flute with wirelength driven mode
void Construct_2d_tree::gen_FR_congestion_map() {
    Tree *flutetree;                        //a struct, defined by Flute library
    Two_pin_element_2d *L_path, *two_pin;
    int *flute_order;
    bboxRouteStateMap = new EdgeColorMap<int>(rr_map.get_gridx(), rr_map.get_gridy(), -1);

    init_2d_map();          //initial congestion map: calculating every edge's capacity
    init_2pin_list();       //initial 2-pin net container
    init_flute();           //initial the information of pin's coordinate and group by net for flute
    flute_mode = NORMAL;	//wirelength driven	mode

    /*assign 0.5 demand to each net*/
#ifdef MESSAGE
    printf("bbox routing start...\n");
#endif	

    //for storing the RSMT which returned by flute
    Flute netRoutingTreeRouter;
    flutetree = (Tree*) calloc(rr_map.get_netNumber(), sizeof(Tree));

    //Get every net's possible RSMT by flute, then use it to calculate the possible congestion
    //In this section, we won't get a real routing result, but a possible congestion information.
    for (int i = 0; i < rr_map.get_netNumber(); ++i)	//i:net id
            {
#ifdef DEBUG_BBOX
        printf("bbox route net %d start...pin_num=%d\n",i,rr_map.get_netPinNumber(i));
#endif

        //call flute to gen steiner tree and put the result in flutetree[]
        netRoutingTreeRouter.routeNet(rr_map.get_nPin(i), flutetree[i]);

        //The total node # in a tree, those nodes include pin and steinor point
        //And it is defined as ((2 * degree of a tree) - 2) by the authors of flute
        flutetree[i].number = 2 * flutetree[i].deg - 2;	//add 0403

        /*2-pin bounding box assign demand 0.5, remember not to repeat the same net*/
        for (int j = 0; j < flutetree[i].number; ++j)	//for all pins and steiner points
                {
            int x1 = (int) flutetree[i].branch[j].x;
            int y1 = (int) flutetree[i].branch[j].y;
            int x2 = (int) flutetree[i].branch[flutetree[i].branch[j].n].x;
            int y2 = (int) flutetree[i].branch[flutetree[i].branch[j].n].y;
            if (!(x1 == x2 && y1 == y2))	//start and end are not the same point
            {
                two_pin = new Two_pin_element_2d();
                two_pin->pin1.x = x1;
                two_pin->pin1.y = y1;
                two_pin->pin2.x = x2;
                two_pin->pin2.y = y2;
                two_pin->net_id = i;
                bbox_2pin_list[i]->push_back(two_pin);
            }
        }

        bbox_route(bbox_2pin_list[i], 0.5);
    }
#ifdef DEBUG1
    printf("bbox routing complete\n");
    print_cap("max");
    print_cap("cur");
#endif	
#ifdef MESSAGE
    printf("L-shaped pattern routing start...\n");
#endif		

    //sort net by their bounding box size, then by their pin number
    vector<const Net*> sort_net;
    for (int i = 0; i < rr_map.get_netNumber(); ++i) {
        sort_net.push_back(&rr_map.get_netList()[i]);
    }
    sort(sort_net.begin(), sort_net.end(), [&]( const Net* a, const Net* b ) {comp_net(a,b);});

    //Now begins the initial routing by pattern routing
    //Edge shifting will also be applied to the routing.
    for (vector<const Net*>::iterator it = sort_net.begin(); it != sort_net.end(); ++it) {
        int netId = (*it)->id;

        flute_order = (int*) malloc(sizeof(int) * (2 * flutetree[netId].deg - 2));

        edge_shifting(&flutetree[netId]);
        global_flutetree = flutetree[netId];
        setup_flute_order(flute_order);
        net_flutetree[netId] = flutetree[netId];

        /*remove demand*/
        bbox_route(bbox_2pin_list[netId], -0.5);

        for (int k = 0; k < flutetree[netId].number; ++k) {
            int j = flute_order[k];
            int x1 = (int) flutetree[netId].branch[j].x;
            int y1 = (int) flutetree[netId].branch[j].y;
            int x2 = (int) flutetree[netId].branch[flutetree[netId].branch[j].n].x;
            int y2 = (int) flutetree[netId].branch[flutetree[netId].branch[j].n].y;
            if (!(x1 == x2 && y1 == y2)) {
                /*choose the L-shape with lower congestion to assign new demand 1*/
                L_path = new Two_pin_element_2d();
                L_pattern_route(x1, y1, x2, y2, L_path, netId);

                /*insert 2pin_path into this net*/
                net_2pin_list[netId]->push_back(L_path);
                update_congestion_map_insert_two_pin_net(L_path);
#ifdef DEBUG_LROUTE
                print_path(*L_path);
#endif
            }
        }
#ifdef FREE
        free(flute_order);
#endif
    }

#ifdef MESSAGE
    printf("generate L-shape congestion map in stage1 successfully\n");
#endif	

#ifdef DEBUG1
    print_cap("cur");
#endif
    cal_max_overflow();
    delete bboxRouteStateMap;
}

//=====================edge shifting=============================
//Return the smaller cost of L-shape path or the only one cost of flap path
double Construct_2d_tree::compute_L_pattern_cost(int x1, int y1, int x2, int y2, int net_id) {
    Two_pin_element_2d path1, path2;
    Monotonic_element max_cong_path1, max_cong_path2;

    if (x1 > x2) {
        swap(x1, x2);
        swap(y1, y2);
    }

    if (x1 < x2 && y1 < y2) {
        max_cong_path1 = L_pattern_max_cong(x1, y1, x2, y2, FRONT, RIGHT, &path1, net_id);
        max_cong_path2 = L_pattern_max_cong(x1, y1, x2, y2, RIGHT, FRONT, &path2, net_id);

        if ((&max_cong_path1) == (compare_cost(&max_cong_path1, &max_cong_path2)))
            return max_cong_path1.total_cost;
        else
            return max_cong_path2.total_cost;
    } else if (x1 < x2 && y1 > y2) {
        max_cong_path1 = L_pattern_max_cong(x1, y1, x2, y2, BACK, RIGHT, &path1, net_id);
        max_cong_path2 = L_pattern_max_cong(x1, y1, x2, y2, RIGHT, BACK, &path2, net_id);

        if ((&max_cong_path1) == (compare_cost(&max_cong_path1, &max_cong_path2)))
            return max_cong_path1.total_cost;
        else
            return max_cong_path2.total_cost;
    } else // vertical or horizontal line
    {
        if (y1 > y2) {
            swap(y1, y2);
        }
        max_cong_path1 = L_pattern_max_cong(x1, y1, x2, y2, FRONT, RIGHT, &path1, net_id);
        return max_cong_path1.total_cost;
    }
}

void Construct_2d_tree::find_saferange(Vertex_flute_ptr a, Vertex_flute_ptr b, int *low, int *high, int dir) {
    Vertex_flute_ptr cur, find;

    //Horizontal edge doing vertical shifting
    if (dir == HOR) {
        cur = a;
        while (cur->type != PIN) {
            find = *(cur->neighbor.begin());
            for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei) {
                if ((*nei)->y > find->y) {
                    find = *nei;
                }
            }
            cur = find;
            if ((cur->y == a->y) || (cur->x != a->x))
                break;
        }
        *high = min(*high, cur->y);

        cur = b;
        while (cur->type != PIN) {
            find = *(cur->neighbor.begin());
            for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)
                if ((*nei)->y > find->y)
                    find = *nei;
            cur = find;
            if ((cur->y == b->y) || (cur->x != b->x))
                break;
        }
        *high = min(*high, cur->y);

        cur = a;
        while (cur->type != PIN) {
            find = *(cur->neighbor.begin());
            for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)
                if ((*nei)->y < find->y)
                    find = *nei;
            cur = find;
            if (cur->y == a->y || cur->x != a->x)
                break;
        }
        *low = max(*low, cur->y);

        cur = b;
        while (cur->type != PIN) {
            find = *(cur->neighbor.begin());
            for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)
                if ((*nei)->y < find->y)
                    find = *nei;
            cur = find;
            if ((cur->y == b->y) || (cur->x != b->x))
                break;
        }
        *low = max(*low, cur->y);
    } else {
        cur = a;
        while (cur->type != PIN) {
            find = *(cur->neighbor.begin());
            for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)
                if ((*nei)->x > find->x)
                    find = *nei;
            cur = find;
            if (cur->x == a->x || cur->y != a->y)	//no neighboring vertex in the right of a
                break;
        }
        *high = min(*high, cur->x);
        cur = b;
        while (cur->type != PIN) {
            find = *(cur->neighbor.begin());
            for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)
                if ((*nei)->x > find->x)
                    find = *nei;
            cur = find;
            if (cur->x == b->x || cur->y != b->y)	//no neighboring vertex in the right of b
                break;
        }
        *high = min(*high, cur->x);
        cur = a;
        while (cur->type != PIN) {
            find = *(cur->neighbor.begin());
            for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)
                if ((*nei)->x < find->x)
                    find = *nei;
            cur = find;
            if (cur->x == a->x || cur->y != a->y)	//no neighboring vertex in the left of a
                break;
        }
        *low = max(*low, cur->x);
        cur = b;
        while (cur->type != PIN) {
            find = *(cur->neighbor.begin());
            for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)
                if ((*nei)->x < find->x)
                    find = *nei;
            cur = find;
            if (cur->x == b->x || cur->y != b->y)	//no neighboring vertex in the left of b
                break;
        }
        *low = max(*low, cur->x);
    }
}

void Construct_2d_tree::merge_vertex(Vertex_flute_ptr keep, Vertex_flute_ptr deleted) {

    for (vector<Vertex_flute_ptr>::iterator nei = deleted->neighbor.begin(); nei != deleted->neighbor.end(); ++nei) {
        if ((*nei) != keep)	//nei is not keep itself
                {
            keep->neighbor.push_back(*nei);	//add deleted's neighbor to keep
            for (int find = 0;; ++find) {
//#ifdef DEBUG_EDGESHIFT
                if (find == (int) (*nei)->neighbor.size()) {
                    printf("wrong in merge_vertex\n");
                    exit(0);
                }
//#endif
                if ((*nei)->neighbor[find]->x == deleted->x && (*nei)->neighbor[find]->y == deleted->y)	//neighbor[find] equals to deleted
                        {
                    (*nei)->neighbor[find] = keep;	//replace its neighbor as keep
                    break;
                }
            }
        }
    }
    deleted->type = DELETED;
}

bool Construct_2d_tree::move_edge(Vertex_flute_ptr a, Vertex_flute_ptr b, int best_pos, int dir) {
    vector<Vertex_flute_ptr> st_pt;
    Vertex_flute_ptr cur, find;
    Vertex_flute_ptr overlap_a, overlap_b;
    int ind1, ind2;

    overlap_a = overlap_b = NULL;
    if (dir == HOR) {
        if (best_pos > a->y)	//move up
                {
            //find all steiner points between a and best_pos
            cur = a;
            while (cur->y < best_pos) {
                find = *(cur->neighbor.begin());
                for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)
                    if ((*nei)->y > find->y)
                        find = *nei;
                cur = find;
                st_pt.push_back(cur);
            }
            //exchange neighb
            for (int i = 0; i < (int) st_pt.size(); ++i) {
                if (st_pt[i]->y < best_pos) {
                    for (ind1 = 0;; ++ind1) {
                        if (!(((a->neighbor[ind1]->x == b->x) && (a->neighbor[ind1]->y == b->y)) || ((a->neighbor[ind1]->x == st_pt[i]->x) && (a->neighbor[ind1]->y == st_pt[i]->y)))) {
                            break;
                        }
                    }
                    for (ind2 = 0;; ++ind2)
                        if (st_pt[i]->neighbor[ind2]->y > st_pt[i]->y)
                            break;
                    for (int j = 0;; ++j)
                        if (a->neighbor[ind1]->neighbor[j] == a) {
                            a->neighbor[ind1]->neighbor[j] = st_pt[i];
                            break;
                        }
                    for (int j = 0;; ++j)
                        if (st_pt[i]->neighbor[ind2]->neighbor[j] == st_pt[i]) {
                            st_pt[i]->neighbor[ind2]->neighbor[j] = a;
                            break;
                        }

                    swap(a->neighbor[ind1], st_pt[i]->neighbor[ind2]);

                    a->y = st_pt[i]->y;
                } else if (st_pt[i]->x == a->x && st_pt[i]->y == best_pos)
                    overlap_a = st_pt[i];
                else
                    break;
            }
            st_pt.clear();
            cur = b;
            while (cur->y < best_pos) {
                find = *(cur->neighbor.begin());
                for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)
                    if ((*nei)->y > find->y)
                        find = *nei;
                cur = find;
                st_pt.push_back(cur);
            }
            for (int i = 0; i < (int) st_pt.size(); ++i) {
                if (st_pt[i]->y < best_pos) {
                    for (ind1 = 0;; ++ind1)
                        if (!((b->neighbor[ind1]->x == a->x && b->neighbor[ind1]->y == a->y) || (b->neighbor[ind1]->x == st_pt[i]->x && b->neighbor[ind1]->y == st_pt[i]->y)))
                            break;
                    for (ind2 = 0;; ++ind2)
                        if (st_pt[i]->neighbor[ind2]->y > st_pt[i]->y)
                            break;

                    for (int j = 0;; ++j)
                        if (b->neighbor[ind1]->neighbor[j] == b) {
                            b->neighbor[ind1]->neighbor[j] = st_pt[i];
                            break;
                        }
                    for (int j = 0;; ++j)
                        if (st_pt[i]->neighbor[ind2]->neighbor[j] == st_pt[i]) {
                            st_pt[i]->neighbor[ind2]->neighbor[j] = b;
                            break;
                        }

                    swap(b->neighbor[ind1], st_pt[i]->neighbor[ind2]);

                    b->y = st_pt[i]->y;
                } else if (st_pt[i]->x == b->x && st_pt[i]->y == best_pos)
                    overlap_b = st_pt[i];
                else
                    break;
            }
            st_pt.clear();

            a->y = best_pos;
            b->y = best_pos;
        } else	//move down
        {
            cur = a;
            while (cur->y > best_pos) {
                find = *(cur->neighbor.begin());
                for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)
                    if ((*nei)->y < find->y)
                        find = *nei;
                cur = find;
                st_pt.push_back(cur);
            }
            for (int i = 0; i < (int) st_pt.size(); ++i) {
                if (st_pt[i]->y > best_pos) {
                    for (ind1 = 0;; ++ind1)
                        if (!((a->neighbor[ind1]->x == b->x && a->neighbor[ind1]->y == b->y) || (a->neighbor[ind1]->x == st_pt[i]->x && a->neighbor[ind1]->y == st_pt[i]->y)))
                            break;
                    for (ind2 = 0;; ++ind2)
                        if (st_pt[i]->neighbor[ind2]->y < st_pt[i]->y)
                            break;
                    for (int j = 0;; ++j)
                        if (a->neighbor[ind1]->neighbor[j] == a) {
                            a->neighbor[ind1]->neighbor[j] = st_pt[i];
                            break;
                        }
                    for (int j = 0;; ++j)
                        if (st_pt[i]->neighbor[ind2]->neighbor[j] == st_pt[i]) {
                            st_pt[i]->neighbor[ind2]->neighbor[j] = a;
                            break;
                        }

                    swap(a->neighbor[ind1], st_pt[i]->neighbor[ind2]);

                    a->y = st_pt[i]->y;
                } else if (st_pt[i]->x == a->x && st_pt[i]->y == best_pos)
                    overlap_a = st_pt[i];
                else
                    break;
            }
            st_pt.clear();
            cur = b;
            while (cur->y > best_pos) {
                find = *(cur->neighbor.begin());
                for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)
                    if ((*nei)->y < find->y)
                        find = *nei;
                cur = find;
                st_pt.push_back(cur);
            }
            for (int i = 0; i < (int) st_pt.size(); ++i) {
                if (st_pt[i]->y > best_pos) {
                    for (ind1 = 0;; ++ind1)
                        if (!((b->neighbor[ind1]->x == a->x && b->neighbor[ind1]->y == a->y) || (b->neighbor[ind1]->x == st_pt[i]->x && b->neighbor[ind1]->y == st_pt[i]->y)))
                            break;
                    for (ind2 = 0;; ++ind2)
                        if (st_pt[i]->neighbor[ind2]->y < st_pt[i]->y)
                            break;

                    for (int j = 0;; ++j)
                        if (b->neighbor[ind1]->neighbor[j] == b) {
                            b->neighbor[ind1]->neighbor[j] = st_pt[i];
                            break;
                        }
                    for (int j = 0;; ++j)
                        if (st_pt[i]->neighbor[ind2]->neighbor[j] == st_pt[i]) {
                            st_pt[i]->neighbor[ind2]->neighbor[j] = b;
                            break;
                        }

                    swap(b->neighbor[ind1], st_pt[i]->neighbor[ind2]);

                    b->y = st_pt[i]->y;
                } else if (st_pt[i]->x == b->x && st_pt[i]->y == best_pos)
                    overlap_b = st_pt[i];
                else
                    break;
            }
            st_pt.clear();

            a->y = best_pos;
            b->y = best_pos;
        }
    } else	//VER
    {
        if (best_pos > a->x)	//move right
                {
            //find all steiner points between a and best_pos
            cur = a;
            while (cur->x < best_pos) {
                find = *(cur->neighbor.begin());
                for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)
                    if ((*nei)->x > find->x)
                        find = *nei;
                cur = find;
                st_pt.push_back(cur);
            }
            //exchange neighbor
            for (int i = 0; i < (int) st_pt.size(); ++i) {
                if (st_pt[i]->x < best_pos) {
                    for (ind1 = 0;; ++ind1)
                        if (!((a->neighbor[ind1]->x == b->x && a->neighbor[ind1]->y == b->y) || (a->neighbor[ind1]->x == st_pt[i]->x && a->neighbor[ind1]->y == st_pt[i]->y)))
                            break;
                    for (ind2 = 0;; ++ind2)
                        if (st_pt[i]->neighbor[ind2]->x > st_pt[i]->x)
                            break;

                    for (int j = 0;; ++j)
                        if (a->neighbor[ind1]->neighbor[j] == a) {
                            a->neighbor[ind1]->neighbor[j] = st_pt[i];
                            break;
                        }
                    for (int j = 0;; ++j)
                        if (st_pt[i]->neighbor[ind2]->neighbor[j] == st_pt[i]) {
                            st_pt[i]->neighbor[ind2]->neighbor[j] = a;
                            break;
                        }

                    swap(a->neighbor[ind1], st_pt[i]->neighbor[ind2]);

                    a->x = st_pt[i]->x;
                } else if (st_pt[i]->x == best_pos && st_pt[i]->y == a->y)
                    overlap_a = st_pt[i];
                else
                    break;
            }
            st_pt.clear();
            cur = b;
            while (cur->x < best_pos) {
                find = *(cur->neighbor.begin());
                for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)
                    if ((*nei)->x > find->x)
                        find = *nei;
                cur = find;
                st_pt.push_back(cur);
            }
            //exchange neighbor
            for (int i = 0; i < (int) st_pt.size(); ++i) {
                if (st_pt[i]->x < best_pos) {
                    for (ind1 = 0;; ++ind1)
                        if (!((b->neighbor[ind1]->x == a->x && b->neighbor[ind1]->y == a->y) || (b->neighbor[ind1]->x == st_pt[i]->x && b->neighbor[ind1]->y == st_pt[i]->y)))
                            break;
                    for (ind2 = 0;; ++ind2)
                        if (st_pt[i]->neighbor[ind2]->x > st_pt[i]->x)
                            break;

                    for (int j = 0;; ++j)
                        if (b->neighbor[ind1]->neighbor[j] == b) {
                            b->neighbor[ind1]->neighbor[j] = st_pt[i];
                            break;
                        }
                    for (int j = 0;; ++j)
                        if (st_pt[i]->neighbor[ind2]->neighbor[j] == st_pt[i]) {
                            st_pt[i]->neighbor[ind2]->neighbor[j] = b;
                            break;
                        }

                    swap(b->neighbor[ind1], st_pt[i]->neighbor[ind2]);

                    b->x = st_pt[i]->x;
                } else if (st_pt[i]->x == best_pos && st_pt[i]->y == b->y)
                    overlap_b = st_pt[i];
                else
                    break;
            }
            st_pt.clear();

            a->x = best_pos;
            b->x = best_pos;
        } else	//move left
        {
            cur = a;
            while (cur->x > best_pos) {
                find = *(cur->neighbor.begin());
                for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)
                    if ((*nei)->x < find->x)
                        find = *nei;
                cur = find;
                st_pt.push_back(cur);
            }
            for (int i = 0; i < (int) st_pt.size(); ++i) {
                if (st_pt[i]->x > best_pos) {
                    for (ind1 = 0;; ++ind1)
                        if (!((a->neighbor[ind1]->x == b->x && a->neighbor[ind1]->y == b->y) || (a->neighbor[ind1]->x == st_pt[i]->x && a->neighbor[ind1]->y == st_pt[i]->y)))
                            break;
                    for (ind2 = 0;; ++ind2)
                        if (st_pt[i]->neighbor[ind2]->x < st_pt[i]->x)
                            break;

                    for (int j = 0;; ++j)
                        if (a->neighbor[ind1]->neighbor[j] == a) {
                            a->neighbor[ind1]->neighbor[j] = st_pt[i];
                            break;
                        }
                    for (int j = 0;; ++j)
                        if (st_pt[i]->neighbor[ind2]->neighbor[j] == st_pt[i]) {
                            st_pt[i]->neighbor[ind2]->neighbor[j] = a;
                            break;
                        }

                    swap(a->neighbor[ind1], st_pt[i]->neighbor[ind2]);

                    a->x = st_pt[i]->x;
                } else if (st_pt[i]->x == best_pos && st_pt[i]->y == a->y)
                    overlap_a = st_pt[i];
                else
                    break;
            }
            st_pt.clear();
            cur = b;
            while (cur->x > best_pos) {
                find = *(cur->neighbor.begin());
                for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)
                    if ((*nei)->x < find->x)
                        find = *nei;
                cur = find;
                st_pt.push_back(cur);
            }
            for (int i = 0; i < (int) st_pt.size(); ++i) {
                if (st_pt[i]->x > best_pos) {
                    for (ind1 = 0;; ++ind1)
                        if (!((b->neighbor[ind1]->x == a->x && b->neighbor[ind1]->y == a->y) || (b->neighbor[ind1]->x == st_pt[i]->x && b->neighbor[ind1]->y == st_pt[i]->y)))
                            break;
                    for (ind2 = 0;; ++ind2)
                        if (st_pt[i]->neighbor[ind2]->x < st_pt[i]->x)
                            break;

                    for (int j = 0;; ++j)
                        if (b->neighbor[ind1]->neighbor[j] == b) {
                            b->neighbor[ind1]->neighbor[j] = st_pt[i];
                            break;
                        }
                    for (int j = 0;; ++j)
                        if (st_pt[i]->neighbor[ind2]->neighbor[j] == st_pt[i]) {
                            st_pt[i]->neighbor[ind2]->neighbor[j] = b;
                            break;
                        }

                    swap(b->neighbor[ind1], st_pt[i]->neighbor[ind2]);

                    b->x = st_pt[i]->x;
                } else if (st_pt[i]->x == best_pos && st_pt[i]->y == b->y)
                    overlap_b = st_pt[i];
                else
                    break;
            }
            st_pt.clear();

            a->x = best_pos;
            b->x = best_pos;
        }
    }

    bool ret = false;
    if (overlap_a != NULL) {
        merge_vertex(overlap_a, a);
        ret = true;
    }
    if (overlap_b != NULL)
        merge_vertex(overlap_b, b);

    return ret;
}

void Construct_2d_tree::traverse_tree(double *ori_cost) {
    double cur_cost, tmp_cost, best_cost;
    int best_pos = 0;
    Vertex_flute_ptr node;

    for (vector<Vertex_flute_ptr>::iterator it_v = vertex_fl.begin(); it_v != vertex_fl.end(); ++it_v) {
        node = *it_v;
        node->visit = 1;

        if (node->type == STEINER && node->neighbor.size() <= 3)	//remove3!!!
                {
            for (vector<Vertex_flute_ptr>::iterator it = node->neighbor.begin(); it != node->neighbor.end(); ++it)
                if ((*it)->visit == 0 && ((*it)->type == STEINER && (*it)->neighbor.size() <= 3))	//!!!remove3!!
                        {
                    int low = 0, high = INT_MAX;
                    Vertex_flute_ptr a = node, b = (*it);

                    if ((*it)->y == node->y)	//horizontal edge (vertical shifting)
                            {
                        find_saferange(a, b, &low, &high, HOR);		//compute safe range
                        best_cost = *ori_cost;
                        cur_cost = (*ori_cost) - compute_L_pattern_cost(a->x, a->y, b->x, b->y, -1);
                        for (int pos = low; pos <= high; ++pos) {
                            tmp_cost = cur_cost + compute_L_pattern_cost(a->x, pos, b->x, pos, -1);
                            if (tmp_cost < best_cost) {
                                best_cost = tmp_cost;
                                best_pos = pos;
                            }
                        }
                        if (best_cost < *ori_cost)	//edge need shifting
                                {
                            move_edge(a, b, best_pos, HOR);
                            //move to best position,exchange steiner points if needed
                            *ori_cost = best_cost;
                        }
                    } else if ((*it)->x == node->x)	//vertical edge (horizontal shifting)
                            {
                        find_saferange(a, b, &low, &high, VER);		//compute safe range
                        best_cost = *ori_cost;
                        cur_cost = (*ori_cost) - compute_L_pattern_cost(a->x, a->y, b->x, b->y, -1);
                        for (int pos = low; pos <= high; ++pos) {
                            tmp_cost = cur_cost + compute_L_pattern_cost(pos, a->y, pos, b->y, -1);
                            if (tmp_cost < best_cost) {
                                best_cost = tmp_cost;
                                best_pos = pos;
                            }
                        }
                        if (best_cost < *ori_cost)	//edge need shifting
                                {
                            move_edge(a, b, best_pos, VER);
                            //move to best position,exchange steiner points if needed
                            *ori_cost = best_cost;
                        }
                    } else
                        continue;
                }
        }
    }
}

void Construct_2d_tree::dfs_output_tree(Vertex_flute_ptr node, Tree *t) {
    node->visit = 1;
    t->branch[t->number].x = node->x;
    t->branch[t->number].y = node->y;
    node->index = t->number;
    (t->number) += 1;
    for (vector<Vertex_flute_ptr>::iterator it = node->neighbor.begin(); it != node->neighbor.end(); ++it) {
        if (((*it)->visit == 0) && ((*it)->type != DELETED)) {
            dfs_output_tree((*it), t);                  //keep tracing deeper vertice
            t->branch[(*it)->index].n = node->index;    //make parsent of target vertex point to current vertex
        }
    }
}

void Construct_2d_tree::edge_shifting(Tree *t) {
    Vertex_flute_ptr new_v;
    double ori_cost;            // the original cost without edge shifting

    ori_cost = 0;
    //creat vertex
    for (int i = 0; i < t->deg; ++i) {
        new_v = new Vertex_flute((int) t->branch[i].x, (int) t->branch[i].y);
        new_v->type = PIN;
        vertex_fl.push_back(new_v);
    }
    for (int i = t->deg; i < 2 * t->deg - 2; ++i) {
        new_v = new Vertex_flute((int) t->branch[i].x, (int) t->branch[i].y);
        new_v->type = STEINER;
        vertex_fl.push_back(new_v);
    }

    //creat edge
    for (int i = 0; i < 2 * (t->deg) - 2; ++i) {
        //skip the vertex if it is the same vertex with its neighbor
        if ((vertex_fl[i]->x == vertex_fl[t->branch[i].n]->x) && (vertex_fl[i]->y == vertex_fl[t->branch[i].n]->y))
            continue;
        vertex_fl[i]->neighbor.push_back(vertex_fl[t->branch[i].n]);
        vertex_fl[t->branch[i].n]->neighbor.push_back(vertex_fl[i]);
        //compute original tree cost
        ori_cost += compute_L_pattern_cost(vertex_fl[i]->x, vertex_fl[i]->y, vertex_fl[t->branch[i].n]->x, vertex_fl[t->branch[i].n]->y, -1);
    }
    std::sort(vertex_fl.begin(), vertex_fl.end(), [&](Vertex_flute_ptr a, Vertex_flute_ptr b) {comp_vertex_fl(a,b);});

    for (int i = 0, j = 1; j < 2 * (t->deg) - 2; ++j) {
        if ((vertex_fl[i]->x == vertex_fl[j]->x) && (vertex_fl[i]->y == vertex_fl[j]->y))	//j is redundant
                {
            vertex_fl[j]->type = DELETED;
            for (vector<Vertex_flute_ptr>::iterator it = vertex_fl[j]->neighbor.begin(); it != vertex_fl[j]->neighbor.end(); ++it) {
                if (((*it)->x != vertex_fl[i]->x) || ((*it)->y != vertex_fl[i]->y))	//not i,add 0430
                        {
                    vertex_fl[i]->neighbor.push_back(*it);
                    for (int k = 0; k < (int) (*it)->neighbor.size(); k++) {
                        if ((*it)->neighbor[k]->x == vertex_fl[i]->x && (*it)->neighbor[k]->y == vertex_fl[i]->y) {
                            (*it)->neighbor[k] = vertex_fl[i];	//modify (*it)'s neighbor to i
                            break;
                        }
                    }
                }
            }
        } else
            i = j;
    }

    for (int i = 0; i < 2 * t->deg - 2; ++i)
        vertex_fl[i]->visit = 0;
    traverse_tree(&ori_cost);	// dfs to find 2 adjacent steiner points(h or v edge) and do edge_shifhting

    //Output the result (2-pin lists) to a Tree structure in DFS order
    //1. make sure every 2-pin list have not been visited
    for (int i = 0; i < 2 * t->deg - 2; ++i) {
        vertex_fl[i]->visit = 0;
    }
    t->number = 0;

    //2. begin to out put the 2-pin lists to a Tree strucuture
    dfs_output_tree(vertex_fl[0], t);
    t->branch[0].n = 0;	//because neighbor of root is not assign in dfs_output_tree()

    //3. free memory resources
    for (int i = 0; i < (int) vertex_fl.size(); ++i)
        delete (vertex_fl[i]);
    vertex_fl.clear();
}
//=====================end edge shifting=============================

/* sort by bounding box size */
void Construct_2d_tree::output_2_pin_list() {
    std::sort(all_two_pin_list.begin(), all_two_pin_list.end(), [&](Two_pin_element *a, Two_pin_element *b) {
        return comp_2pin_net(a,b);
    });
    std::sort(two_pin_list.begin(), two_pin_list.end(), [&](Two_pin_element_2d *a, Two_pin_element_2d *b) {
        return comp_2pin_net_from_path(a,b);
    });
}

/*used for stage2
 return the maximum overflow=cur_cap/max_cap
 update 0307:only assign max_cap to cur_3d_map
 */
void Construct_2d_tree::output_3d_map() {
    int i, j, k;

    //assign max_cap to cur_3d_map
    for (i = 0; i < rr_map.get_gridx() - 1; ++i)
        for (j = 0; j < rr_map.get_gridy(); ++j)
            for (k = 0; k < rr_map.get_layerNumber(); ++k) {
                cur_map_3d[i][j][k].edge_list[RIGHT]->max_cap = rr_map.capacity(k, i, j, i + 1, j);
            }
    for (i = 0; i < rr_map.get_gridx(); ++i)
        for (j = 0; j < rr_map.get_gridy() - 1; ++j)
            for (k = 0; k < rr_map.get_layerNumber(); ++k) {
                cur_map_3d[i][j][k].edge_list[FRONT]->max_cap = rr_map.capacity(k, i, j, i, j + 1);
            }

#ifdef DEBUG1
    print_cap_3d("max");
    print_cap_3d("cur");
#endif	

}

/*stage 1: contruct 2d steiner tree
 output 2-pin list to stage2
 return max_overflow;
 */

Construct_2d_tree::Construct_2d_tree(RoutingParameters& routingparam, ParameterSet& param, RoutingRegion& rr, std::function<void(int i, int j, int dir)> pre_evaluate_congestion_cost_fp) :
        pre_evaluate_congestion_cost_fp { pre_evaluate_congestion_cost_fp }, parameter_set { param }, routing_parameter { routingparam }, rr_map { rr } {

    par_ind = 0;

    dir_array[4][2] = { {0, 1}, {0, -1}, {-1, 0}, {1, 0}}; //FRONT,BACK,LEFT,RIGHT
    Jr2JmDirArray[4] = {0, 1, 3, 2}; //FRONT,BACK,LEFT,RIGHT <-> North, South, East, West

    Multisource_multisink_mazeroute* mazeroute_in_range = NULL;

    int via_cost = 3;
    double max_congestion_factor = 1.0;

    cache = nullptr;
    /***********************
     * Global Variable End
     * ********************/

    factor = 1.0;
    exponent = 5.0;
    WL_Cost = 1.0;
    adjust_value = 0;

    cur_iter = -1;                  // current iteration ID.
    //edgeIterCounter = new EdgeColorMap<int>(rr_map.get_gridx(), rr_map.get_gridy(), -1);

    used_cost_flag = FASTROUTE_COST;    // cost function type, i.e., HISTORY_COST, HISTORY_MADEOF_COST, MADEOF_COST, FASTROUTE_COST
    total_overflow = 0;             // used by post-processing

    readLUT();                      // Function in flute, functino: unknown

    /* TroyLee: NetDirtyBit Counter */
    NetDirtyBit = vector<bool>(rr_map.get_netNumber(), true);
    /* TroyLee: End */

    allocate_coor_array();          // Make an 2D coordinate array which contains the (x, y) information

    if (routing_parameter.get_monotonic_en()) {
        allocate_monotonic();           // Allocate the memory for storing the data while searching monotonic path
                                        // 1. A 2D array that stores max congestion
                                        // 2. A 2D array that stores parent (x,y) during finding monotonic path
    }

    gen_FR_congestion_map();        // Generate congestion map by flute, then route all nets by L-shap pattern routing with
                                    // congestion information from this map. After that, apply edge shifting to the result
                                    // to get the initial solution.

    cal_total_wirelength();         // The report value is the sum of demand on every edge

    allocate_gridcell();            //question: I don't know what this for. (jalamorm, 07/10/31)

    //Make a 2-pin net list without group by net
    for (int i = 0; i < rr_map.get_netNumber(); ++i) {
        for (int j = 0; j < (int) net_2pin_list[i]->size(); ++j) {
            two_pin_list.push_back((*net_2pin_list[i])[j]);
        }
    }

    reallocate_two_pin_list(true);
    cache = new EdgePlane<CacheEdge>(rr_map.get_gridx(), rr_map.get_gridy(), CacheEdge());
    mazeroute_in_range = new Multisource_multisink_mazeroute(&this);

    int cur_overflow = -1;
    used_cost_flag = HISTORY_COST;
    BOXSIZE_INC = routing_parameter->get_init_box_size_p2();

    for (cur_iter = 1, done_iter = cur_iter; cur_iter <= routing_parameter->get_iteration_p2(); ++cur_iter, done_iter = cur_iter)   //do n-1 times
            {
        cout << "\033[31mIteration:\033[m " << cur_iter << endl;

        factor = (1.0 - exp(-5 * exp(-(0.1 * cur_iter))));

        WL_Cost = factor;
        via_cost = static_cast<int>(4 * factor);
        adjust_value = cur_iter * (1.25 + 3 * factor); //tuned for experimant

#ifdef MESSAGE
        cout << "Parameters - Factor: " << factor
        << ", Via_Cost: " << via_cost
        << ", Box Size: " << BOXSIZE_INC + cur_iter - 1 << endl;
#endif

        pre_evaluate_congestion_cost();

        //route_all_2pin_net(false);
        route_all_2pin_net();

        cur_overflow = cal_max_overflow();
        cal_total_wirelength();

        if (cur_overflow == 0)
            break;

        reallocate_two_pin_list(true);

#ifdef MESSAGE
        printMemoryUsage("Memory Usage:");
#endif

        if (cur_overflow <= routing_parameter->get_overflow_threshold()) {
            break;
        }
        BOXSIZE_INC += routing_parameter->get_box_size_inc_p2();
    }

    output_2_pin_list();    //order:bbox�p

#ifdef FREE
    free_memory_con2d();
#endif
#ifdef MESSAGE
    cout<<"================================================================"<<endl;
    cout<<"===                   Enter Post Processing                  ==="<<endl;
    cout<<"================================================================"<<endl;
#endif

}

