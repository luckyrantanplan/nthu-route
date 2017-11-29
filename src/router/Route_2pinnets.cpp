#include "Route_2pinnets.h"
#include "Range_router.h"
#include "Construct_2d_tree.h"
#include "../flute/flute-ds.h"
#include "../misc/geometry.h"
#include "../util/traversemap.h"
#include "../grdb/RoutingRegion.h"
#include <cmath>
#include <algorithm>

using namespace std;

Route_2pinnets::Route_2pinnets(Construct_2d_tree& construct_2d_tree, RangeRouter& rangerouter) :
        gridcell { rr_map.get_gridx(), rr_map.get_gridy(), Point_fc() }, //
        traverseMap { }, //
        terminalMap { }, //
        dirTransferTable { 1, 0, 3, 2 }, //
        construct_2d_tree { construct_2d_tree }, //
        rr_map { construct_2d_tree.rr_map }, //
        rangerouter { rangerouter } {
    ;
}

void Route_2pinnets::allocate_gridcell() {

    for (int x = rr_map.get_gridx() - 1; x >= 0; --x) {
        for (int y = rr_map.get_gridy() - 1; y >= 0; --y) {
            gridcell.vertex(x, y).x = x;
            gridcell.vertex(x, y).y = y;
        }
    }

#ifdef MESSAGE
    printf("initialize gridcell successfully\n");
#endif
}

void Route_2pinnets::init_gridcell() {
    for (int i = 0; i < rr_map.get_gridx(); ++i) {
        for (int j = 0; j < rr_map.get_gridy(); ++j) {
            gridcell.vertex(i, j).points.clear();
        }
    }
    int length = construct_2d_tree.two_pin_list.size();
    for (int i = 0; i < length; ++i) {
        //add pin1
        Two_pin_element_2d& two_pin = construct_2d_tree.two_pin_list[i];
        int cur_x = two_pin.pin1.x;
        int cur_y = two_pin.pin1.y;
        gridcell.vertex(cur_x, cur_y).points.push_back(two_pin);
        //add pin2
        cur_x = two_pin.pin2.x;
        cur_y = two_pin.pin2.y;
        gridcell.vertex(cur_x, cur_y).points.push_back(two_pin);
    }
}

void Route_2pinnets::route_all_2pin_net() {
    vector<Point_fc *> all_cells;

    init_gridcell();

    rangerouter.define_interval();
    rangerouter.divide_grid_edge_into_interval();

    rangerouter.specify_all_range(gridcell);
}

void Route_2pinnets::reset_c_map_used_net_to_one() {
    RoutedNetTable::iterator iter;

    for (int i = rr_map.get_gridx() - 2; i >= 0; --i) {
        for (int j = rr_map.get_gridy() - 1; j >= 0; --j) {
            RoutedNetTable* table = &(construct_2d_tree.congestionMap2d.edge(i, j, DIR_EAST).used_net);
            for (iter = table->begin(); iter != table->end(); ++iter) {
                (*table)[iter->first] = 1;
            }
        }
    }

    for (int i = rr_map.get_gridx() - 1; i >= 0; --i) {
        for (int j = rr_map.get_gridy() - 2; j >= 0; --j) {
            RoutedNetTable* table = &(construct_2d_tree.congestionMap2d.edge(i, j, DIR_NORTH).used_net);
            for (iter = table->begin(); iter != table->end(); ++iter) {
                (*table)[iter->first] = 1;
            }
        }
    }
}

//set terminal type to c_map_2d
//void set_c_map_terminal_type(int net_id)
void Route_2pinnets::put_terminal_color_on_colormap(int net_id) {
    for (int i = rr_map.get_nPin(net_id).size() - 1; i >= 0; --i) {
        int xx = rr_map.get_nPin(net_id)[i]->get_tileX();
        int yy = rr_map.get_nPin(net_id)[i]->get_tileY();
        terminalMap.color(xx, yy) = net_id;
    }
}

//return: one-degree terminal, non-one-degree terminal, one-degree nonterminal, steiner point, two-degree (dir)
int Route_2pinnets::determine_is_terminal_or_steiner_point(int xx, int yy, int dir, int net_id) {
    int find_dir = 0;
    static int gridxMinusOne = rr_map.get_gridx() - 1;
    static int gridyMinusOne = rr_map.get_gridy() - 1;

    dir = dirTransferTable[dir];

    if (terminalMap.color(xx, yy) == net_id) {
        for (int i = 3; i >= 0; --i) {
            if (i != dir) {
                if ((i == 3 && xx >= gridxMinusOne) || (i == 2 && xx <= 0) || (i == 1 && yy <= 0) || (i == 0 && yy >= gridyMinusOne))
                    continue;
                else {
                    if (construct_2d_tree.congestionMap2d.edge(xx, yy, i).lookupNet(net_id)) {
                        return -3;
                    }
                }
            }
        }
        return -4;
    } else {
        int other_passed_edge = 0;
        for (int i = 3; i >= 0; --i) {
            if (i != dir) {
                if ((i == 3 && xx >= gridxMinusOne) || (i == 2 && xx <= 0) || (i == 1 && yy <= 0) || (i == 0 && yy >= gridyMinusOne))
                    continue;
                else {
                    if (construct_2d_tree.congestionMap2d.edge(xx, yy, i).lookupNet(net_id)) {
                        ++other_passed_edge;
                        find_dir = i;
                        if ((other_passed_edge & 0x02) != 0)
                            return -1;
                    }
                }
            }
        }
        if (other_passed_edge == 0)
            return -2;
        else
            return find_dir;
    }
}

void Route_2pinnets::bfs_for_find_two_pin_list(Coordinate_2d *start_coor, int net_id, bool insert_to_branch) {
    vector<Coordinate_2d *> queue;
    vector<int> parent;
    vector<int> branch_ind;
    vector<Coordinate_2d *> branch_xy;
    vector<int> branch_n;
    int head_index, tail_index, dir, ori_dir;
    int x, y, xx, yy, i;
    queue.push_back(start_coor);
    parent.push_back(-1);
    head_index = 0;
    tail_index = 1;

    traverseMap.color(queue[head_index]->x, queue[head_index]->y) = net_id;

    if (insert_to_branch) {
        branch_xy.push_back(queue[head_index]);
        branch_n.push_back(0);
        branch_ind.push_back(0);
    }

    while (head_index != tail_index) {
        for (i = 0; i <= 3; ++i) {
            if (i == parent[head_index])
                continue;
            x = queue[head_index]->x;
            y = queue[head_index]->y;
            dir = i;
            const std::array<int, 2>& dira = construct_2d_tree.dir_array[dir];
            if (x + dira[0] >= 0 && x + dira[0] < rr_map.get_gridx() && //
                    y + dira[1] >= 0 && y + dira[1] < rr_map.get_gridy()) {
                if (construct_2d_tree.congestionMap2d.edge(x, y, dir).lookupNet(net_id)) {
                    Two_pin_element_2d* two_pin;
                    if (construct_2d_tree.two_pin_list_size >= (int) construct_2d_tree.two_pin_list.size()) {
                        two_pin = new Two_pin_element_2d();
                    } else {
                        two_pin = construct_2d_tree.two_pin_list[construct_2d_tree.two_pin_list_size];
                    }
                    two_pin->pin1.x = queue[head_index]->x;
                    two_pin->pin1.y = queue[head_index]->y;
                    two_pin->net_id = net_id;
                    two_pin->path.clear();
                    two_pin->path.push_back(queue[head_index]);
                    while (1) {
                        traverseMap.color(x, y) = net_id;
                        xx = x + dira[0];
                        yy = y + dira[1];
                        ori_dir = dir;
                        dir = determine_is_terminal_or_steiner_point(xx, yy, dir, net_id);
                        two_pin->path.push_back(&construct_2d_tree.coor_array[xx][yy]);
                        if (dir < 0 && dir != -2) {
                            if (traverseMap.color(xx, yy) != net_id) {
                                two_pin->pin2.x = xx;
                                two_pin->pin2.y = yy;
                                if (construct_2d_tree.two_pin_list_size >= (int) construct_2d_tree.two_pin_list.size())
                                    construct_2d_tree.two_pin_list.push_back(two_pin);
                                construct_2d_tree.two_pin_list_size++;

                                if (insert_to_branch) {
                                    branch_xy.push_back(&construct_2d_tree.coor_array[xx][yy]);
                                    branch_n.push_back(branch_ind[head_index]);
                                }

                                if (dir != -4) {
                                    queue.push_back(&construct_2d_tree.coor_array[xx][yy]);
                                    if (ori_dir == FRONT)
                                        parent.push_back(BACK);
                                    else if (ori_dir == BACK)
                                        parent.push_back(FRONT);
                                    else if (ori_dir == RIGHT)
                                        parent.push_back(LEFT);
                                    else
                                        parent.push_back(RIGHT);

                                    if (insert_to_branch)
                                        branch_ind.push_back(branch_xy.size() - 1);

                                    tail_index++;
                                }
                                traverseMap.color(xx, yy) = net_id;
                            } else {
                                construct_2d_tree.update_congestion_map_remove_two_pin_net(two_pin);
                                construct_2d_tree.NetDirtyBit[two_pin->net_id] = true;
                                if (construct_2d_tree.two_pin_list_size >= (int) construct_2d_tree.two_pin_list.size()) {
                                    two_pin->path.clear();
                                    delete (two_pin);
                                }
                            }
                            break;
                        } else if (dir == -2) {
                            construct_2d_tree.update_congestion_map_remove_two_pin_net(two_pin);
                            construct_2d_tree.NetDirtyBit[two_pin->net_id] = true;
                            if (construct_2d_tree.two_pin_list_size >= (int) construct_2d_tree.two_pin_list.size()) {
                                two_pin->path.clear();
                                delete (two_pin);
                            }
                            break;
                        } else {
                            x = xx;
                            y = yy;
                        }
                    }
                }
            }
        }
        head_index++;
    }

    if (insert_to_branch) {
        Tree& tree = construct_2d_tree.net_flutetree[net_id];
        if (tree.number < (int) branch_xy.size()) {
            free(tree.branch);
            tree.branch = (Branch *) malloc(branch_xy.size() * sizeof(Branch));
        }

        tree.number = branch_xy.size();
        for (i = tree.number - 1; i >= 0; --i) {
            tree.branch[i].x = branch_xy[i]->x;
            tree.branch[i].y = branch_xy[i]->y;
            tree.branch[i].n = branch_n[i];
        }
    }
}

void Route_2pinnets::reallocate_two_pin_list(bool insert_to_branch) {
    int dirty_count = 0;
    traverseMap = new VertexColorMap<int>(rr_map.get_gridx(), rr_map.get_gridy(), -1);
    terminalMap = new VertexColorMap<int>(rr_map.get_gridx(), rr_map.get_gridy(), -1);

    reset_c_map_used_net_to_one();

    construct_2d_tree.two_pin_list_size = 0;

    int usedTwoPinListSize = 0;
    for (int twoPinListPos = 0; twoPinListPos < (int) construct_2d_tree.two_pin_list.size(); ++twoPinListPos) {
        if (construct_2d_tree.NetDirtyBit[construct_2d_tree.two_pin_list[twoPinListPos]->net_id] == false) {
            if (usedTwoPinListSize != twoPinListPos) {
                swap(construct_2d_tree.two_pin_list[twoPinListPos], construct_2d_tree.two_pin_list[usedTwoPinListSize]);
                ++usedTwoPinListSize;
            } else {
                ++usedTwoPinListSize;
            }
        }
    }

    construct_2d_tree.two_pin_list_size = usedTwoPinListSize;

    for (int netId = 0; netId < rr_map.get_netNumber(); ++netId) {
        if (construct_2d_tree.NetDirtyBit[netId] == true) {
            put_terminal_color_on_colormap(netId);
            int xx = rr_map.get_nPin(netId)[0]->get_tileX();
            int yy = rr_map.get_nPin(netId)[0]->get_tileY();
            Coordinate_2d* start_coor = &construct_2d_tree.coor_array[xx][yy];

            bfs_for_find_two_pin_list(start_coor, netId, insert_to_branch);

            construct_2d_tree.NetDirtyBit[netId] = false;
            ++dirty_count;
        }
    }

    for (int i = construct_2d_tree.two_pin_list.size() - 1; i >= construct_2d_tree.two_pin_list_size; --i) {
        delete (construct_2d_tree.two_pin_list[i]);
        construct_2d_tree.two_pin_list.pop_back();
    }

    delete traverseMap;
    delete terminalMap;
}
