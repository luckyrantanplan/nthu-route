#include "MM_mazeroute.h"
#include "../misc/geometry.h"
#include "../misc/debug.h"
#include "Post_processing.h"
#include <iostream>
#include <climits>
#include <stack>
#include <queue>
#include "../grdb/RoutingRegion.h"
#include "Construct_2d_tree.h"

using namespace Jm;
using namespace std;

Multisource_multisink_mazeroute::Vertex_mmm::Vertex_mmm(Coordinate_2d& xy) :
        coor(xy), visit(-1) {
}

Multisource_multisink_mazeroute::Multisource_multisink_mazeroute(Construct_2d_tree& construct_2d_tree) :
        construct_2d_tree { construct_2d_tree }, gridxMinusOne(construct_2d_tree.rr_map.get_gridx() - 1), gridyMinusOne(construct_2d_tree.rr_map.get_gridy() - 1) {
    /*allocate space for mmm_map*/

    RoutingRegion& rr_map = construct_2d_tree.rr_map;

    net_tree.resize(rr_map.get_netNumber());

    mmm_map = new VertexPlane<MMM_element>(rr_map.get_gridx(), rr_map.get_gridy(), MMM_element());

    //initialization
    for (int i = 0; i < rr_map.get_gridx(); ++i) {
        for (int j = 0; j < rr_map.get_gridy(); ++j) {
            mmm_map->vertex(i, j).coor = &construct_2d_tree.coor_array[i][j];
        }
    }

    visit_counter = 0;
    dst_counter = 0;
}

Multisource_multisink_mazeroute::~Multisource_multisink_mazeroute() {

    delete mmm_map;

}

/*recursively traverse parent in maze_routing_map to find path*/
void Multisource_multisink_mazeroute::trace_back_to_find_path_2d(MMM_element *end_point) {
    MMM_element *cur_pos;

    cur_pos = end_point;
    while (1) {
        if (cur_pos == NULL)
            break;
        element->path.push_back(cur_pos->coor);
        cur_pos = cur_pos->parent;
    }
}

//store new 2pins and adjust dfs tree
void Multisource_multisink_mazeroute::adjust_twopin_element() {

    Coordinate_2d* new_pin1 = element->path.front();
    Coordinate_2d* new_pin2 = element->path.back();
    element->pin1 = construct_2d_tree.coor_array[new_pin1->x][new_pin1->y];
    element->pin2 = construct_2d_tree.coor_array[new_pin2->x][new_pin2->y];

    int flag = 0;
    for (auto it = pin1_v->neighbor.begin(); it != pin1_v->neighbor.end(); ++it) {
        if (&(*it) == pin2_v) {
            pin1_v->neighbor.erase(it);
            flag = 1;
            break;
        }
    }
    assert(flag == 1);

    flag = 0;
    for (auto it = pin2_v->neighbor.begin(); it != pin2_v->neighbor.end(); ++it) {
        if (&(*it) == pin1_v) {
            pin2_v->neighbor.erase(it);
            flag = 1;
            break;
        }
    }
    assert(flag == 1);

    int net_id = element->net_id;
    Vertex_mmm* v1 = NULL;
    Vertex_mmm* v2 = NULL;

    vector<Vertex_mmm *>& vect = (*net_tree)[net_id];
    for (vector<Vertex_mmm *>::iterator it = vect.begin(); it != vect.end() && (v1 == nullptr || v2 == nullptr); ++it) {
        if ((*it)->coor == new_pin1) {
            v1 = (*it);
        } else if ((*it)->coor == new_pin2) {
            v2 = (*it);
        }
    }
    assert(v1 != NULL);
    assert(v2 != NULL);

    v1->neighbor.push_back(v2);
    v2->neighbor.push_back(v1);
}

void Multisource_multisink_mazeroute::find_subtree(Vertex_mmm *v, int mode) {
    v->visit = visit_counter;

    if (mode == 0) {
        MMM_element *cur = &mmm_map->vertex(v->coor.x, v->coor.y);
        cur->reachCost = 0;
        cur->distance = 0;
        cur->via_num = 0;
        cur->parent = NULL;
        cur->visit = visit_counter;
        pqueue->insert(cur);
    } else {
        mmm_map->vertex(v->coor.x, v->coor.y).dst = dst_counter;
    }
    for (vector<Vertex_mmm *>::iterator it = v->neighbor.begin(); it != v->neighbor.end(); ++it) {
        if ((*it)->visit != visit_counter)
            find_subtree((*it), mode);
    }
}

void Multisource_multisink_mazeroute::clear_net_tree() {
    for (int i = 0; i < construct_2d_tree.rr_map.get_netNumber(); ++i) {
        int length = (*net_tree)[i].size();
        for (int j = 0; j < length; ++j)
            delete ((*net_tree)[i][j]);
    }

    delete net_tree;
    net_tree = new vector<vector<Vertex_mmm *> >(construct_2d_tree.rr_map.get_netNumber(), vector<Vertex_mmm *>(0));
}

void Multisource_multisink_mazeroute::setup_pqueue() {

    int cur_net = element->net_id;
    if ((*net_tree)[cur_net].empty()) {
        Tree* t = &construct_2d_tree.net_flutetree[cur_net];
        for (int i = 0; i < t->number; ++i) {
            Branch& branch = t->branch[i];
            Vertex_mmm * vertex = new Vertex_mmm(construct_2d_tree.coor_array[(int) branch.x][(int) branch.y]);
            (*net_tree)[cur_net].push_back(vertex);
        }
        for (int i = 0; i < t->number; ++i) {
            Vertex_mmm* a = (*net_tree)[cur_net][i];
            Vertex_mmm* b = (*net_tree)[cur_net][t->branch[i].n];
            if (a->coor.x == b->coor.x && a->coor.y == b->coor.y)
                continue;
            a->neighbor.push_back(b);
            b->neighbor.push_back(a);
        }
    }

    pqueue->clear();

//find pin1 and pin2 in tree
    pin1_v = pin2_v = NULL;
    for (vector<Vertex_mmm *>::iterator it = (*net_tree)[cur_net].begin(); (it != (*net_tree)[cur_net].end()) && (pin1_v == NULL || pin2_v == NULL); ++it) {
        if ((*it)->coor.x == element->pin1.x && (*it)->coor.y == element->pin1.y) {
            pin1_v = (*it);
            pin1_v->visit = visit_counter;
        } else if ((*it)->coor.x == element->pin2.x && (*it)->coor.y == element->pin2.y) {
            pin2_v = (*it);
            pin2_v->visit = visit_counter;
        }
    }

    assert(pin1_v != NULL);
    assert(pin2_v != NULL);

    find_subtree(pin1_v, 0);	//source
    find_subtree(pin2_v, 1);	//destination
}

void Multisource_multisink_mazeroute::bfsSetColorMap(int x, int y) {
    int net_id = element->net_id;
    stack<pair<int, int> > Q;

    Q.push(make_pair(x, y));
    while (!Q.empty()) {
        x = Q.top().first;
        y = Q.top().second;
        Q.pop();
        mmm_map->vertex(x, y).walkableID = visit_counter;

        for (int dir = 3; dir >= 0; --dir) {
            if ((dir == 3 && x >= gridxMinusOne) || (dir == 2 && x <= 0) || (dir == 1 && y <= 0) || (dir == 0 && y >= gridyMinusOne))
                continue;
            else {
                DirectionType dirType = static_cast<DirectionType>(construct_2d_tree.Jr2JmDirArray[dir]);
                CacheEdge& edge = construct_2d_tree.cache->edge(x, y, dirType);
                if (edge.MMVisitFlag != visit_counter && construct_2d_tree.congestionMap2d.edge(x, y, dirType).lookupNet(net_id)) {
                    edge.MMVisitFlag = visit_counter;
                    Q.push(make_pair(x + construct_2d_tree.dir_array[dir][0], y + construct_2d_tree.dir_array[dir][1]));
                }
            }
        }
    }
}
bool Multisource_multisink_mazeroute::mm_maze_route_p2(Two_pin_element_2d &ielement, double bound_cost, int bound_distance, int bound_via_num, Coordinate_2d start, Coordinate_2d end) {
    int pre_dir = 0;
    bool find_path_flag = false;
    MMM_element* cur_pos = NULL;
    MMM_element* next_pos = NULL;
    MMM_element* sink_pos = NULL;
    element->path.clear();
    element = ielement;
    boundary_l = start.x;
    boundary_b = start.y;
    boundary_r = end.x;
    boundary_t = end.y;
    setup_pqueue();
    putNetOnColorMap();

    for (int x = boundary_l; x <= boundary_r; ++x) {
        for (int y = boundary_b; y <= boundary_t; ++y) {
            mmm_map->vertex(x, y).walkableID = visit_counter;
        }
    }

    while (!pqueue.empty()) {
        cur_pos = pqueue.top();
        pqueue.pop();

        if (cur_pos->parent != NULL) {
            pre_dir = Coordinate_2d::get_direction_simple(*cur_pos->coor, *cur_pos->parent->coor);
            //We only need horizontal or vertical information of the direction,
            //so we can &0x02 first.
            //pre_dir &= 0x02;
        }

        for (int i = 3; i >= 0; --i) {
            if ((i == 3 && cur_pos->coor.x >= gridxMinusOne) || (i == 2 && cur_pos->coor.x <= 0) || (i == 1 && cur_pos->coor.y <= 0) || (i == 0 && cur_pos->coor.y >= gridyMinusOne))
                continue;
            else {
                next_pos = &mmm_map->vertex(cur_pos->coor.x + construct_2d_tree.dir_array[i][0], cur_pos->coor.y + construct_2d_tree.dir_array[i][1]);

                if (next_pos != cur_pos->parent && next_pos->walkableID == visit_counter) {
                    DirectionType dirType = static_cast<DirectionType>(construct_2d_tree.Jr2JmDirArray[i]);
                    double reachCost = cur_pos->reachCost;
                    int total_distance = cur_pos->distance;
                    int via_num = cur_pos->via_num;
                    bool addDistance = false;
                    CacheEdge& edge = construct_2d_tree.cache->edge(cur_pos->coor.x, cur_pos->coor.y, dirType);
                    if (edge.MMVisitFlag != visit_counter) {
                        reachCost += edge.cost;
                        ++total_distance;
                        addDistance = true;
                    }

                    if (((i & 0x02) != pre_dir) && (cur_pos->parent != NULL)) {
                        via_num += 3;
                        if (addDistance) {
                            total_distance += 3;
                            reachCost += construct_2d_tree.via_cost;
                        }
                    }

                    bool needUpdate = false;
                    if (next_pos->visit != visit_counter) {
                        if (construct_2d_tree.smaller_than_lower_bound(reachCost, total_distance, via_num, bound_cost, bound_distance, bound_via_num)) {
                            needUpdate = true;
                        }
                    } else {
                        if (construct_2d_tree.smaller_than_lower_bound(reachCost, total_distance, via_num, next_pos->reachCost, next_pos->distance, next_pos->via_num)) {
                            needUpdate = true;
                        }
                    }

                    if (needUpdate == true) {
                        next_pos->parent = cur_pos;
                        next_pos->reachCost = reachCost;
                        next_pos->distance = total_distance;
                        next_pos->via_num = via_num;
                        next_pos->visit = visit_counter;

                        if (next_pos->dst == dst_counter) {
                            bound_cost = reachCost;
                            bound_distance = total_distance;
                            bound_via_num = via_num;
                            sink_pos = next_pos;
                        } else {
                            pqueue->update(next_pos);
                        }
                    }
                }
            }
        }            //end of direction for-loop

        if (sink_pos != NULL) {
            find_path_flag = true;
            trace_back_to_find_path_2d(sink_pos);
            adjust_twopin_element();
            break;
        }
    }

    ++visit_counter;
    ++dst_counter;
    return find_path_flag;
}

bool Multisource_multisink_mazeroute::mm_maze_route_p3(Two_pin_element_2d *ielement, double bound_cost, int bound_distance, int bound_via_num, Coordinate_2d start, Coordinate_2d end) {
    int pre_dir = 0;
    bool find_path_flag = false;
    MMM_element* cur_pos = NULL;
    MMM_element* next_pos = NULL;
    MMM_element* sink_pos = NULL;

    element->path.clear();
    element = ielement;
    boundary_l = start.x;
    boundary_b = start.y;
    boundary_r = end.x;
    boundary_t = end.y;
    setup_pqueue();
    putNetOnColorMap();

    for (int x = boundary_l; x <= boundary_r; ++x) {
        for (int y = boundary_b; y <= boundary_t; ++y) {
            mmm_map->vertex(x, y).walkableID = visit_counter;
        }
    }

    while (!pqueue->empty()) {
        cur_pos = pqueue->top();
        pqueue->pop();

        if (cur_pos->parent != NULL) {
            pre_dir = construct_2d_tree.get_direction_2d_simple(cur_pos->coor, cur_pos->parent->coor);
            //We only need horizontal or vertical information of the direction,
            //so we can &0x02 first.
            //pre_dir &= 0x02;
        }

        for (int i = 3; i >= 0; --i) {
            if ((i == 3 && cur_pos->coor.x >= gridxMinusOne) || (i == 2 && cur_pos->coor.x <= 0) || (i == 1 && cur_pos->coor.y <= 0) || (i == 0 && cur_pos->coor.y >= gridyMinusOne))
                continue;
            else {
                const std::array<int, 2>& dira = construct_2d_tree.dir_array[i];
                next_pos = &mmm_map->vertex(cur_pos->coor.x + dira[0], cur_pos->coor.y + dira[1]);

                if (next_pos != cur_pos->parent && next_pos->walkableID == visit_counter) {
                    DirectionType dirType = static_cast<DirectionType>(construct_2d_tree.Jr2JmDirArray[i]);
                    double reachCost = cur_pos->reachCost;
                    int total_distance = cur_pos->distance;
                    int via_num = cur_pos->via_num;
                    CacheEdge& edge = construct_2d_tree.cache->edge(cur_pos->coor.x, cur_pos->coor.y, dirType);
                    if ((edge.MMVisitFlag != visit_counter) && (edge.cost != 0.0)) {
                        reachCost += edge.cost;
                        ++total_distance;
                    }

                    if (((i & 0x02) != pre_dir) && (cur_pos->parent != NULL)) {
                        via_num += 3;
                    }

                    bool needUpdate = false;
                    if (next_pos->visit != visit_counter) {
                        if (smaller_than_lower_bound(reachCost, total_distance, via_num, bound_cost, bound_distance, bound_via_num)) {
                            needUpdate = true;
                        }
                    } else {
                        if (smaller_than_lower_bound(reachCost, total_distance, via_num, next_pos->reachCost, next_pos->distance, next_pos->via_num)) {
                            needUpdate = true;
                        }
                    }

                    if (needUpdate == true) {
                        next_pos->parent = cur_pos;
                        next_pos->reachCost = reachCost;
                        next_pos->distance = total_distance;
                        next_pos->via_num = via_num;
                        next_pos->visit = visit_counter;

                        if (next_pos->dst == dst_counter) {
                            bound_cost = reachCost;
                            bound_distance = total_distance;
                            bound_via_num = via_num;
                            sink_pos = next_pos;
                        } else {
                            pqueue.push(*next_pos);
                        }
                    }
                }
            }
        }            //end of direction for-loop

        if (sink_pos != NULL) {
            find_path_flag = true;
            trace_back_to_find_path_2d(sink_pos);
            adjust_twopin_element();
            break;
        }
    }

    ++visit_counter;
    ++dst_counter;
    return find_path_flag;
}

inline
void Multisource_multisink_mazeroute::putNetOnColorMap() {
    bfsSetColorMap(pin1_v->coor.x, pin1_v->coor.y);
    bfsSetColorMap(pin2_v->coor.x, pin2_v->coor.y);
}

bool Multisource_multisink_mazeroute::smaller_than_lower_bound(double total_cost, int distance, int via_num, double bound_cost, int bound_distance, int bound_via_num) {
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

