#include "MM_mazeroute.h"

#include <boost/heap/detail/stable_heap.hpp>
#include <boost/multi_array/base.hpp>
#include <boost/multi_array/multi_array_ref.hpp>
#include <boost/multi_array/subarray.hpp>
#include <sys/types.h>
#include <cassert>
#include <cstdio>
#include <iostream>
#include <iterator>
#include <stack>

#include "../flute/flute-ds.h"
#include "flute4nthuroute.h"
#include "../grdb/EdgePlane.h"
#include "../grdb/RoutingRegion.h"
#include "../spdlog/common.h"
#include "../spdlog/details/spdlog_impl.h"
#include "../spdlog/logger.h"
#include "Congestion.h"
#include "Construct_2d_tree.h"

#define SPDLOG_TRACE_ON
#include "../spdlog/spdlog.h"

namespace NTHUR {

using namespace std;

Multisource_multisink_mazeroute::Vertex_mmm::Vertex_mmm(const Coordinate_2d& xy) :
        coor(xy), neighbor { }, visit(-1) {
}

Multisource_multisink_mazeroute::Multisource_multisink_mazeroute(Construct_2d_tree& construct_2d_tree, Congestion& congestion) :
        construct_2d_tree { construct_2d_tree }, //
        congestion { congestion }, //
        mmm_map { boost::extents[congestion.congestionMap2d.getXSize()][congestion.congestionMap2d.getYSize()] }, //
        element { }, //
        pin1_v { }, //
        pin2_v { } {
    /*allocate space for mmm_map*/
    log_sp = spdlog::get("NTHUR");
    net_tree.resize(construct_2d_tree.rr_map.get_netNumber());

    //initialization

    for (u_int32_t i = 0; i < mmm_map.size(); ++i) {
        for (u_int32_t j = 0; j < mmm_map[0].size(); ++j) {
            mmm_map[i][j].coor.set(i, j);
            mmm_map[i][j].parent = &mmm_map[i][j];
        }
    }

    visit_counter = 0;
    dst_counter = 0;

}

/*recursively traverse parent in maze_routing_map to find path*/
void Multisource_multisink_mazeroute::trace_back_to_find_path_2d(MMM_element *end_point) {
    MMM_element *cur_pos = end_point;
    while (1) {
        SPDLOG_TRACE(log_sp, "cur_pos->coor {}", cur_pos->coor.toString());
        element->path.push_back(cur_pos->coor);
        if (cur_pos == cur_pos->parent) {
            break;
        }
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

void Multisource_multisink_mazeroute::find_subtree(Vertex_mmm& v, int mode) {
    v.visit = visit_counter;

    if (mode == 0) {
        MMM_element& cur = mmm_map[v.coor.x][v.coor.y];
        cur.reachCost = 0;
        cur.distance = 0;
        cur.via_num = 0;
        cur.parent = &cur;
        cur.visit = visit_counter;
        cur.handle = pqueue.push(&cur);

        SPDLOG_TRACE(log_sp, "find_subtree cur {}", cur.toString());

    } else {
        mmm_map[v.coor.x][v.coor.y].dst = dst_counter;
    }
    for (Vertex_mmm * neighbor : v.neighbor) {
        if (neighbor->visit != visit_counter)
            find_subtree(*neighbor, mode);
    }
}

void Multisource_multisink_mazeroute::clear_net_tree() {

    net_tree.clear();
    net_tree.resize(construct_2d_tree.rr_map.get_netNumber());

}

void Multisource_multisink_mazeroute::setup_pqueue() {

    int cur_net = element->net_id;
    vector<Vertex_mmm>& vertexV = net_tree[cur_net];
    if (vertexV.empty()) {
        const TreeFlute& t = construct_2d_tree.net_flutetree[cur_net];

        if (log_sp->level() == spdlog::level::trace) {
            log_sp->trace(t.plot());
        }
        vertexV.reserve(t.number); // avoid re allocation that could invalidate pointer

        std::unordered_map<Coordinate_2d, int> indexmap;
        indexmap.reserve(t.number);

        for (int i = 0; i < t.number; ++i) {

            Coordinate_2d c { (int) t.branch[i].x, (int) t.branch[i].y };
            bool inserted = indexmap.insert( { c, static_cast<int>(vertexV.size()) }).second;
            if (inserted) {
                vertexV.emplace_back(c);
            }
        }

        for (int i = 0; i < t.number; ++i) {
            Branch b = t.branch[i];
            Coordinate_2d c1 { (int) b.x, (int) b.y };
            Coordinate_2d c2 { (int) t.branch[b.n].x, (int) t.branch[b.n].y };
            Vertex_mmm& v1 = vertexV.at(indexmap.at(c1));
            Vertex_mmm& v2 = vertexV.at(indexmap.at(c2));
            if (v1.coor != v2.coor) {
                v1.neighbor.push_back(&v2);
                v2.neighbor.push_back(&v1);
            }
        }
    }

    while (!pqueue.empty()) {
        MMM_element& cur_pos = *pqueue.top();
        pqueue.pop();
        cur_pos.resetHandle();
    }

//find pin1 and pin2 in tree
    pin1_v = nullptr;
    pin2_v = nullptr;
    for (Vertex_mmm& vert : vertexV) {
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

    find_subtree(*pin1_v, 0);	//source
    find_subtree(*pin2_v, 1);	//destination
}

void Multisource_multisink_mazeroute::bfsSetColorMap(const Coordinate_2d& c1) {
    int net_id = element->net_id;
    stack<Coordinate_2d> Q;

    Q.push(c1);
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
    SPDLOG_TRACE(log_sp, "setup_pqueue();");
    setup_pqueue();
    SPDLOG_TRACE(log_sp, " putNetOnColorMap();");
    putNetOnColorMap();

    for (int x = boundary_l; x <= boundary_r; ++x) {
        for (int y = boundary_b; y <= boundary_t; ++y) {
            mmm_map[x][y].walkableID = visit_counter;
        }
    }

    while (!pqueue.empty()) {
        MMM_element& cur_pos = *pqueue.top();

        SPDLOG_TRACE(log_sp, "*pqueue.top(); {}", cur_pos.toString());

        pqueue.pop();
        cur_pos.resetHandle();
        for (EdgePlane<Edge_2d>::Handle& h : congestion.congestionMap2d.neighbors(cur_pos.coor)) {

            MMM_element& next_pos = mmm_map[h.vertex().x][h.vertex().y];

            SPDLOG_TRACE(log_sp, "next_pos {}", next_pos.toString());

            SPDLOG_TRACE(log_sp, "h.vertex() {} h.edge() {}", h.vertex().toString(), h.edge().toString());

            if (&next_pos != cur_pos.parent && next_pos.walkableID == visit_counter) {

                SPDLOG_TRACE(log_sp, "&next_pos != cur_pos.parent && next_pos.walkableID == visit_counter");

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

                    if (!cur_pos.parent->coor.isAligned(h.vertex())) { // other and parent are not align
                        via_num += 3;
                        if (addDistance) {
                            total_distance += 3;
                            reachCost += congestion.via_cost;
                        }
                    }

                } else { // version==3
                    if ((h.edge().MMVisitFlag != visit_counter) && (h.edge().cost != 0.0)) {
                        reachCost += h.edge().cost;
                        ++total_distance;
                    }

                    if (!cur_pos.parent->coor.isAligned(h.vertex())) { // other and parent are not align
                        via_num += 3;
                    }
                }

                bool needUpdate = false;
                SPDLOG_TRACE(log_sp, "bool needUpdate ?");
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
                    SPDLOG_TRACE(log_sp, "needUpdate next_pos {}", next_pos.toString());

                    SPDLOG_TRACE(log_sp, "next_pos.dst {} == dst_counter {}", next_pos.dst, dst_counter);
                    if (next_pos.dst == dst_counter) {
                        bound_cost = reachCost;
                        bound_distance = total_distance;
                        bound_via_num = via_num;
                        sink_pos = &next_pos;
                    } else {
                        if (next_pos.handle.node_ != nullptr) {
                            pqueue.update(next_pos.handle);
                        } else {
                            next_pos.handle = pqueue.push(&next_pos);
                        }
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

    SPDLOG_TRACE(log_sp, "ielement {}", ielement.toString());

    return find_path_flag;
}

inline
void Multisource_multisink_mazeroute::putNetOnColorMap() {
    bfsSetColorMap(pin1_v->coor);
    bfsSetColorMap(pin2_v->coor);
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

} // namespace NTHUR
