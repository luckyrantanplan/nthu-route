#include "Route_2pinnets.h"

#include <boost/multi_array/base.hpp>
#include <boost/multi_array/multi_array_ref.hpp>
#include <boost/multi_array/subarray.hpp>
#include <stdlib.h>
#include <sys/types.h>
#include <algorithm>
#include <iterator>
#include <queue>
#include <utility>

#include "../flute/flute-ds.h"
#include "../grdb/EdgePlane.h"
#include "../grdb/RoutingComponent.h"
#include "../grdb/RoutingRegion.h"
#include "../misc/geometry.h"
#include "Congestion.h"
#include "Construct_2d_tree.h"

using namespace std;

Route_2pinnets::Route_2pinnets(Construct_2d_tree& construct_2d_tree, RangeRouter& rangerouter, Congestion& congestion) :
        gridcell { boost::extents[rr_map.get_gridx()][rr_map.get_gridy()] }, //
        colorMap { boost::extents[rr_map.get_gridx()][rr_map.get_gridy()] }, //
        dirTransferTable { 1, 0, 3, 2 }, //
        construct_2d_tree { construct_2d_tree }, //
        rr_map { construct_2d_tree.rr_map }, //
        rangerouter { rangerouter }, //
        congestion { congestion } {
    ;
}

void Route_2pinnets::allocate_gridcell() {

    for (u_int32_t x = 0; x < gridcell.size(); ++x) {
        for (u_int32_t y = 0; y < gridcell[0].size(); ++y) {
            gridcell[x][y].set(x, y);
        }
    }

#ifdef MESSAGE
    printf("initialize gridcell successfully\n");
#endif
}

void Route_2pinnets::init_gridcell() {
    for (u_int32_t x = 0; x < gridcell.size(); ++x) {
        for (u_int32_t y = 0; y < gridcell[0].size(); ++y) {
            gridcell[x][y].points.clear();
        }
    }
    int length = construct_2d_tree.two_pin_list.size();
    for (int i = 0; i < length; ++i) {
        //add pin1
        Two_pin_element_2d& two_pin = construct_2d_tree.two_pin_list[i];
        int cur_x = two_pin.pin1.x;
        int cur_y = two_pin.pin1.y;
        gridcell[cur_x][cur_y].points.push_back(&two_pin);
        //add pin2
        cur_x = two_pin.pin2.x;
        cur_y = two_pin.pin2.y;
        gridcell[cur_x][cur_y].points.push_back(&two_pin);
    }
}

void Route_2pinnets::route_all_2pin_net() {
    init_gridcell();
    rangerouter.define_interval();
    rangerouter.divide_grid_edge_into_interval();
    rangerouter.specify_all_range(gridcell);
}

void Route_2pinnets::reset_c_map_used_net_to_one() {

    congestion.congestionMap2d.foreach([](Edge_2d& edge) {
        for (auto& routeNetTable : edge.used_net) {
            edge.used_net[routeNetTable.first]=1;
        }
    });

}

//set terminal type to c_map_2d
//void set_c_map_terminal_type(int net_id)
void Route_2pinnets::put_terminal_color_on_colormap(int net_id) {
    for (const Pin* pin : rr_map.get_nPin(net_id)) {
        colorMap[pin->get_tileX()][pin->get_tileY()].terminal = net_id;
    }
}

//return: one-degree terminal, non-one-degree terminal, one-degree nonterminal, steiner point, two-degree (dir)
Coordinate_2d Route_2pinnets::determine_is_terminal_or_steiner_point(Coordinate_2d& c, Coordinate_2d& head, int net_id, PointType& pointType) {

    Coordinate_2d result;
    if (colorMap[c.x][c.y].terminal == net_id) {
        for (const Coordinate_2d& cr : Coordinate_2d::dir_array()) {
            const Coordinate_2d cc = c + cr;
            if (cc != head) {
                if (congestion.congestionMap2d.isVertexInside(cc)) {
                    if (congestion.congestionMap2d.edge(cc, c).lookupNet(net_id)) {
                        pointType = severalDegreeTerminal;
                        return result;
                    }
                }
            }
        }
        pointType = oneDegreeTerminal;

    } else {
        int other_passed_edge = 0;
        for (const Coordinate_2d& cr : Coordinate_2d::dir_array()) {
            const Coordinate_2d cc = c + cr;
            if (cc != head) {
                if (congestion.congestionMap2d.isVertexInside(cc)) {
                    if (congestion.congestionMap2d.edge(cc, c).lookupNet(net_id)) {
                        ++other_passed_edge;
                        if ((other_passed_edge & 0x02) != 0) {
                            pointType = steinerPoint;
                            return -1;
                        }
                        result = cc;
                    }
                }
            }
        }
        if (other_passed_edge == 0) {
            pointType = oneDegreeNonterminal;

        } else {
            pointType = twoDegree;

        }
    }
    return result;
}

void Route_2pinnets::bfs_for_find_two_pin_list(Coordinate_2d start_coor, int net_id) {

    vector<BranchClass> branch_xy;

    std::queue<ElementQueue> queue;
    uint32_t index = 0;
    queue.emplace(start_coor, Coordinate_2d(-1, -1), index);
    ++index;
    {
        Coordinate_2d& head = queue.front().coor;
        colorMap[head.x][head.y].traverse = net_id;

        branch_xy.emplace_back(head, queue.front().index);
    }

    while (!queue.empty()) {
        for (const Coordinate_2d& cr : Coordinate_2d::dir_array()) {
            ElementQueue& headElement = queue.front();
            Coordinate_2d head = headElement.coor;
            Coordinate_2d c = head + cr;

            if (c != headElement.parent && congestion.congestionMap2d.isVertexInside(c)) {
                Edge_2d& edge = congestion.congestionMap2d.edge(head, c);
                if (edge.lookupNet(net_id)) {
                    Two_pin_element_2d two_pin;

                    two_pin.pin1 = head;
                    two_pin.net_id = net_id;
                    two_pin.path.push_back(head);
                    bool exit_loop = true;
                    while (!exit_loop) {
                        exit_loop = true;
                        colorMap[head.x][head.y].traverse = net_id;
                        two_pin.path.push_back(c);
                        PointType pointType;
                        Coordinate_2d nextc = determine_is_terminal_or_steiner_point(c, head, net_id, pointType);

                        switch (pointType) {
                        case oneDegreeTerminal: {
                            if (colorMap[c.x][c.y].traverse != net_id) {
                                two_pin.pin2 = c;
                                construct_2d_tree.two_pin_list.push_back(two_pin);

                                branch_xy.emplace_back(c, headElement.index);

                                colorMap[c.x][c.y].traverse = net_id;
                            } else {

                                congestion.update_congestion_map_remove_two_pin_net(two_pin);
                                construct_2d_tree.NetDirtyBit[two_pin.net_id] = true;

                            }
                            break;
                        }
                        case severalDegreeTerminal:
                        case steinerPoint: {
                            if (colorMap[c.x][c.y].traverse != net_id) {
                                two_pin.pin2 = c;
                                construct_2d_tree.two_pin_list.push_back(two_pin);

                                branch_xy.emplace_back(c, headElement.index);
                                queue.emplace(c, head, index);
                                ++index;

                                colorMap[c.x][c.y].traverse = net_id;
                            } else {
                                congestion.update_congestion_map_remove_two_pin_net(two_pin);
                                construct_2d_tree.NetDirtyBit[two_pin.net_id] = true;
                            }
                            break;
                        }
                        case oneDegreeNonterminal: {
                            congestion.update_congestion_map_remove_two_pin_net(two_pin);
                            construct_2d_tree.NetDirtyBit[two_pin.net_id] = true;

                            break;
                        }
                        case twoDegree: {
                            head = c;
                            c = nextc;
                            exit_loop = false;
                            break;
                        }
                        }

                    }
                }
            }
        }
        queue.pop();
    }

    Tree& tree = construct_2d_tree.net_flutetree[net_id];
    if (tree.number < (int) branch_xy.size()) {
        free(tree.branch);
        tree.branch = (Branch *) malloc(branch_xy.size() * sizeof(Branch));
    }

    tree.number = branch_xy.size();
    for (int i = 0; i < tree.number; ++i) {
        tree.branch[i].x = branch_xy[i].x;
        tree.branch[i].y = branch_xy[i].y;
        tree.branch[i].n = branch_xy[i].n;
    }

}

void Route_2pinnets::reallocate_two_pin_list() {

    for (u_int32_t i = 0; i < colorMap.num_elements(); ++i) {
        colorMap.data()[i].set(-1, -1);
    }

    reset_c_map_used_net_to_one();

    vector<Two_pin_element_2d>& v = construct_2d_tree.two_pin_list;
    // erase-remove idiom
    v.erase(std::remove_if(v.begin(), v.end(), [& ](const Two_pin_element_2d& pin) {
        return construct_2d_tree.NetDirtyBit[pin.net_id];
    }), v.end());

    for (int netId = 0; netId < rr_map.get_netNumber(); ++netId) {
        if (construct_2d_tree.NetDirtyBit[netId]) {
            put_terminal_color_on_colormap(netId);
            int xx = rr_map.get_nPin(netId)[0]->get_tileX();
            int yy = rr_map.get_nPin(netId)[0]->get_tileY();

            bfs_for_find_two_pin_list(Coordinate_2d(xx, yy), netId);

            construct_2d_tree.NetDirtyBit[netId] = false;

        }
    }

}
