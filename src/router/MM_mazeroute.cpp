#include "MM_mazeroute.h"

#include <array>
#include <cassert>
#include <iterator>
#include <stack>
#include <utility>

#include "../flute/flute-ds.h"
#include "../grdb/RoutingRegion.h"
#include "Congestion.h"
#include "Construct_2d_tree.h"

using namespace std;

Multisource_multisink_mazeroute::Vertex_mmm::Vertex_mmm(const Coordinate_2d& xy) :
        coor(xy), visit(-1) {
}

Multisource_multisink_mazeroute::Multisource_multisink_mazeroute(Construct_2d_tree& construct_2d_tree, Congestion& congestion) :
        construct_2d_tree { construct_2d_tree }, //
        congestion { congestion }, //
        mmm_map { boost::extents[congestion.congestionMap2d.getXSize()][congestion.congestionMap2d.getYSize()] }, //
        element { }, //
        pin1_v { }, //
        pin2_v { } {
    /*allocate space for mmm_map*/

    net_tree.resize(construct_2d_tree.rr_map.get_netNumber());

    //initialization

    for (u_int32_t i = 0; i < mmm_map.size(); ++i) {
        for (u_int32_t j = 0; j < mmm_map[0].size(); ++j) {
            mmm_map[i][j].coor.set(i, j);
        }
    }

    visit_counter = 0;
    dst_counter = 0;
}

Multisource_multisink_mazeroute::~Multisource_multisink_mazeroute() {

}

/*recursively traverse parent in maze_routing_map to find path*/
void Multisource_multisink_mazeroute::trace_back_to_find_path_2d(MMM_element *end_point) {
    MMM_element *cur_pos;

    cur_pos = end_point;
    while (1) {
        if (cur_pos == nullptr)
            break;
        element->path.push_back(cur_pos->coor);
        cur_pos = cur_pos->parent;
    }
}

//store new 2pins and adjust dfs tree
void Multisource_multisink_mazeroute::adjust_twopin_element() {

    Coordinate_2d& new_pin1 = element->path.front();
    Coordinate_2d& new_pin2 = element->path.back();
    element->pin1 = new_pin1;
    element->pin2 = new_pin2;

    int flag = 0;
    for (auto it = pin1_v->neighbor.begin(); it != pin1_v->neighbor.end(); ++it) {
        if ((*it) == pin2_v) {
            pin1_v->neighbor.erase(it);
            flag = 1;
            break;
        }
    }
    assert(flag == 1);

    flag = 0;
    for (auto it = pin2_v->neighbor.begin(); it != pin2_v->neighbor.end(); ++it) {
        if ((*it) == pin1_v) {
            pin2_v->neighbor.erase(it);
            flag = 1;
            break;
        }
    }
    assert(flag == 1);

    int net_id = element->net_id;
    Vertex_mmm* v1 = nullptr;
    Vertex_mmm* v2 = nullptr;

    vector<Vertex_mmm>& vect = net_tree[net_id];
    for (vector<Vertex_mmm>::iterator it = vect.begin(); it != vect.end() && (v1 == nullptr || v2 == nullptr); ++it) {
        if (it->coor == new_pin1) {
            v1 = &(*it);
        } else if (it->coor == new_pin2) {
            v2 = &(*it);
        }
    }
    assert(v1 != nullptr);
    assert(v2 != nullptr);

    v1->neighbor.push_back(v2);
    v2->neighbor.push_back(v1);
}

void Multisource_multisink_mazeroute::find_subtree(Vertex_mmm *v, int mode) {
    v->visit = visit_counter;

    if (mode == 0) {
        MMM_element *cur = &mmm_map[v->coor.x][v->coor.y];
        cur->reachCost = 0;
        cur->distance = 0;
        cur->via_num = 0;
        cur->parent = nullptr;
        cur->visit = visit_counter;
        cur->handle = pqueue.push(cur);
    } else {
        mmm_map[v->coor.x][v->coor.y].dst = dst_counter;
    }
    for (Vertex_mmm * neighbor : v->neighbor) {
        if (neighbor->visit != visit_counter)
            find_subtree(neighbor, mode);
    }
}

void Multisource_multisink_mazeroute::clear_net_tree() {

    net_tree.clear();
    net_tree.resize(construct_2d_tree.rr_map.get_netNumber());
}

void Multisource_multisink_mazeroute::setup_pqueue() {

    int cur_net = element->net_id;
    if (net_tree[cur_net].empty()) {
        Tree& t = construct_2d_tree.net_flutetree[cur_net];
        net_tree[cur_net].reserve(t.number); // avoid re allocation that could invalidate pointer
        for (int i = 0; i < t.number; ++i) {
            Branch& branch = t.branch[i];

            net_tree[cur_net].push_back(Vertex_mmm(Coordinate_2d((int) branch.x, (int) branch.y)));
        }
        for (int i = 0; i < t.number; ++i) {
            Vertex_mmm* a = &net_tree[cur_net][i];
            Vertex_mmm* b = &net_tree[cur_net][t.branch[i].n];
            if (a->coor.x == b->coor.x && a->coor.y == b->coor.y)
                continue;
            a->neighbor.push_back(b);
            b->neighbor.push_back(a);
        }
    }

    pqueue.clear();

//find pin1 and pin2 in tree
    pin1_v = nullptr;
    pin2_v = nullptr;
    for (Vertex_mmm& vert : net_tree[cur_net]) {
        if (vert.coor == element->pin1) {
            pin1_v = &vert;
            pin1_v->visit = visit_counter;
        } else if (vert.coor == element->pin2) {
            pin2_v = &vert;
            pin2_v->visit = visit_counter;
        }

        if (pin1_v != nullptr && pin2_v != nullptr) {
            break;
        }

    }

    assert(pin1_v != nullptr);
    assert(pin2_v != nullptr);

    find_subtree(pin1_v, 0);	//source
    find_subtree(pin2_v, 1);	//destination
}

void Multisource_multisink_mazeroute::bfsSetColorMap(int x, int y) {
    int net_id = element->net_id;
    stack<Coordinate_2d> Q;

    Q.push(Coordinate_2d(x, y));
    while (!Q.empty()) {
        Coordinate_2d c = Q.top();

        Q.pop();
        mmm_map[c.x][c.y].walkableID = visit_counter;

        for (EdgePlane<Edge_2d>::Handle& h : congestion.congestionMap2d.neighbors(c)) {
            if (h.edge().MMVisitFlag != visit_counter && h.edge().lookupNet(net_id)) {
                h.edge().MMVisitFlag = visit_counter;
                Q.push(h.vertex());
            }

        }
    }
}
bool Multisource_multisink_mazeroute::mm_maze_route_p(Two_pin_element_2d &ielement, double bound_cost, int bound_distance, int bound_via_num, Coordinate_2d& start, Coordinate_2d& end, int version) {

    bool find_path_flag = false;

    MMM_element* sink_pos = nullptr;
    element = &ielement;
    element->path.clear();
    int boundary_l = start.x;
    int boundary_b = start.y;
    int boundary_r = end.x;
    int boundary_t = end.y;
    setup_pqueue();
    putNetOnColorMap();

    for (int x = boundary_l; x <= boundary_r; ++x) {
        for (int y = boundary_b; y <= boundary_t; ++y) {
            mmm_map[x][y].walkableID = visit_counter;
        }
    }

    while (!pqueue.empty()) {
        MMM_element& cur_pos = *pqueue.top();
        pqueue.pop();

        if (cur_pos.parent != nullptr) {

        }

        for (EdgePlane<Edge_2d>::Handle& h : congestion.congestionMap2d.neighbors(cur_pos.coor)) {

            MMM_element& next_pos = mmm_map[h.vertex().x][h.vertex().y];

            if (&next_pos != cur_pos.parent && next_pos.walkableID == visit_counter) {

                double reachCost = cur_pos.reachCost;
                int total_distance = cur_pos.distance;
                int via_num = cur_pos.via_num;
                bool addDistance = false;

                if (version == 2) {

                    if (h.edge().MMVisitFlag != visit_counter) {
                        reachCost += h.edge().cost;
                        ++total_distance;
                        addDistance = true;
                    }

                    if (cur_pos.parent != nullptr) {
                        if (cur_pos.parent->coor.x != h.vertex().x && cur_pos.parent->coor.y != h.vertex().y) { // other and parent are not align
                            via_num += 3;
                            if (addDistance) {
                                total_distance += 3;
                                reachCost += congestion.via_cost;
                            }
                        }
                    }
                } else { // version==3
                    if ((h.edge().MMVisitFlag != visit_counter) && (h.edge().cost != 0.0)) {
                        reachCost += h.edge().cost;
                        ++total_distance;
                    }
                    if (cur_pos.parent != nullptr) {
                        if (cur_pos.parent->coor.x != h.vertex().x && cur_pos.parent->coor.y != h.vertex().y) { // other and parent are not align
                            via_num += 3;
                        }
                    }

                }

                bool needUpdate = false;
                if (next_pos.visit != visit_counter) {
                    if (smaller_than_lower_bound(reachCost, total_distance, via_num, bound_cost, bound_distance, bound_via_num)) {
                        needUpdate = true;
                    }
                } else {
                    if (smaller_than_lower_bound(reachCost, total_distance, via_num, next_pos.reachCost, next_pos.distance, next_pos.via_num)) {
                        needUpdate = true;
                    }
                }

                if (needUpdate == true) {
                    next_pos.parent = &cur_pos;
                    next_pos.reachCost = reachCost;
                    next_pos.distance = total_distance;
                    next_pos.via_num = via_num;
                    next_pos.visit = visit_counter;

                    if (next_pos.dst == dst_counter) {
                        bound_cost = reachCost;
                        bound_distance = total_distance;
                        bound_via_num = via_num;
                        sink_pos = &next_pos;
                    } else {

                        pqueue.update(next_pos.handle);
                    }

                }
            }
        }            //end of direction for-loop

        if (sink_pos != nullptr) {
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

