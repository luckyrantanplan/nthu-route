#include "Range_router.h"

#include <algorithm>
#include <array>
#include <initializer_list>
#include <iterator>

#include "../grdb/EdgePlane.h"
#include "../grdb/RoutingRegion.h"
#include "Congestion.h"
#include "Construct_2d_tree.h"
#include "MM_mazeroute.h"
#include "MonotonicRouting.h"
#include "parameter.h"
#include "Post_processing.h"

using namespace std;

bool RangeRouter::double_equal(double a, double b) {
    double diff = a - b;
    if (diff > 0.00001 || diff < -0.00001)
        return false;
    else
        return true;
}

/*sort grid_edge in decending order*/
bool RangeRouter::comp_grid_edge(const Grid_edge_element& a, const Grid_edge_element& b) {
    return congestion.congestionMap2d.edge(a.grid, a.c2).congestion() > congestion.congestionMap2d.edge(b.grid, b.c2).congestion();
}

/*
 determine INTERVAL_NUM(10) intervals between min and max,
 and also compute average congestion value (sum of demand / sum of capacity)
 */
void RangeRouter::define_interval() {

    Congestion::Statistic s = congestion.stat_congestion();

    interval_list[0].begin_value = s.max;

    for (u_int32_t i = 1; i < interval_list.size(); ++i) {
        interval_list[i].begin_value = s.max - ((double) i / interval_list.size()) * (s.max - 1.0);
        interval_list[i - 1].end_value = interval_list[i].begin_value;
    }
    interval_list[interval_list.size() - 1].end_value = 1.;
    for (Interval_element& ele : interval_list) {
        ele.grid_edge_vector.clear();
    }
#ifdef MESSAGE
    printf("interval value: ");
    for (int i=0; i<intervalCount; ++i)
    printf("%lf ",interval_list[i].begin_value);
    printf("%lf ",interval_list[intervalCount-1].end_value);
    putchar('\n');
#endif
}

void RangeRouter::insert_to_interval(Coordinate_2d coor_2d, Coordinate_2d c2) {
    double cong_value = congestion.congestionMap2d.edge(coor_2d, c2).congestion();
    if (cong_value > 1) {
        for (int i = interval_list.size() - 1; i >= 0; --i) {
            Interval_element& ele = interval_list[i];
            if (((cong_value < ele.begin_value) || double_equal(cong_value, ele.begin_value)) && //
                    cong_value > ele.end_value) {
                ele.grid_edge_vector.push_back(Grid_edge_element(coor_2d, c2));
                return;
            }
        }
    }
}

void RangeRouter::divide_grid_edge_into_interval() {

    for (int i = 0; i < congestion.congestionMap2d.getXSize() - 1; ++i) {
        for (int j = 0; j < congestion.congestionMap2d.getYSize(); ++j) {
            insert_to_interval(Coordinate_2d { i, j }, Coordinate_2d { i + 1, j });
        }
    }
    for (int i = 0; i < congestion.congestionMap2d.getXSize(); ++i) {
        for (int j = 0; j < congestion.congestionMap2d.getYSize() - 1; ++j) {
            insert_to_interval(Coordinate_2d { i, j }, Coordinate_2d { i, j + 1 });
        }

    }

#ifdef DEBUG_INTERVAL_LIST	
    int grid_edge_in_interval_num = 0;
    for (int i = 0; i < intervalCount; ++i)
    {
        grid_edge_in_interval_num += interval_list[i].grid_edge_vector.size();
    }
    if (grid_edge_in_interval_num != num_of_grid_edge)
    {
        printf("Mismatch: in interval %d , true %d\n",grid_edge_in_interval_num,num_of_grid_edge);
    }
#endif
}

void RangeRouter::expand_range(Coordinate_2d c1, Coordinate_2d c2, int interval_index) {

//(x1, y1) (x2, y2) are neighbor, so we can do the following operation
    int x1 = min(c1.x, c2.x);
    int x2 = max(c1.x, c2.x);
    int y1 = min(c1.y, c2.y);
    int y2 = max(c1.y, c2.y);

    Coordinate_2d start, end;

//obtain the expanded boundary. This is the very first expand, and the expand unit is 10.
    start.x = max(x1 - EXPAND_RANGE_SIZE, 0);
    RoutingRegion& rr_map = construct_2d_tree.rr_map;
    end.x = min(x2 + EXPAND_RANGE_SIZE, rr_map.get_gridx() - 1);
    start.y = max(y1 - EXPAND_RANGE_SIZE, 0);
    end.y = min(y2 + EXPAND_RANGE_SIZE, rr_map.get_gridy() - 1);

    double total_cong = 0;
    int edge_num = 0;
//Obtain the total congestion value and edge number of RIGHT edges.
    for (int i = start.x; i < end.x; ++i) {
        for (int j = start.y; j <= end.y; ++j) {
            total_cong += congestion.congestionMap2d.edge(i, j, DIR_EAST).congestion();
            edge_num++;
            colorMap[i][j].expand = interval_index;
        }
    }

//Set "visited" flag to the grid cells on the right most column
    for (int j = start.y; j <= end.y; ++j) {
        colorMap[end.x][j].expand = interval_index;
    }

//Obtain the total congestion value and edge number of FRONT edges.
    for (int i = start.x; i <= end.x; ++i) {
        for (int j = start.y; j < end.y; ++j) {
            total_cong += congestion.congestionMap2d.edge(i, j, DIR_NORTH).congestion();
            edge_num++;
        }
    }

    double avg_cong = total_cong / edge_num;
    while ((avg_cong > interval_list[interval_index].end_value) && //
            (avg_cong - interval_list[interval_index].end_value > 0.01)) {
        Coordinate_2d cur_start, cur_end;
        cur_start.x = max(start.x - EXPAND_RANGE_INC, 0);
        cur_end.x = min(end.x + EXPAND_RANGE_INC, rr_map.get_gridx() - 1);
        cur_start.y = max(start.y - EXPAND_RANGE_INC, 0);
        cur_end.y = min(end.y + EXPAND_RANGE_INC, rr_map.get_gridy() - 1);

        if (cur_start.x == 0 && cur_end.x == rr_map.get_gridx() - 1 && cur_start.y == 0 && cur_end.y == rr_map.get_gridy() - 1) {
#ifdef DEBUG_INTERVAL_LIST
            printf("for interval %d, its range is equal to the grid size, %d %d , %d %d \n",interval_index,cur_start.x,cur_start.y,cur_end.x,cur_end.y);
            printf("from edge %d %d -> %d %d \n",x1,y1,x2,y2);
#endif
            start = cur_start;
            end = cur_end;
            break;
        }
        EdgePlane<Edge_2d>& congestionMap2d = congestion.congestionMap2d;
//Below here are four for loops, those for loops update the congestion condition
        for (int i = cur_start.x; i < cur_end.x; ++i) {
            if (cur_start.y != start.y) {
                int j = cur_start.y;
                total_cong += congestionMap2d.edge(i, cur_start.y, DIR_EAST).congestion();
                edge_num++;
                colorMap[i][j].expand = interval_index;
            }
            if (cur_end.y != end.y) {
                int j = cur_end.y;
                total_cong += congestionMap2d.edge(i, cur_end.y, DIR_EAST).congestion();
                edge_num++;
                colorMap[i][j].expand = interval_index;
            }
        }
        for (int j = start.y; j <= end.y; ++j) {
            if (cur_start.x != start.x) {
                int i = cur_start.x;
                total_cong += congestionMap2d.edge(cur_start.x, j, DIR_EAST).congestion();
                edge_num++;
            }
            if (cur_end.x != end.x) {
                int i = end.x; // BUG ??
                total_cong += congestionMap2d.edge(end.x, j, DIR_EAST).congestion();
                edge_num++;
            }
        }
        for (int j = cur_start.y; j < cur_end.y; ++j) {
            if (cur_start.x != start.x) {
                int i = cur_start.x;
                total_cong += congestionMap2d.edge(i, j, DIR_NORTH).congestion();
                edge_num++;
                colorMap[i][j].expand = interval_index;
            }
            if (cur_end.x != end.x) {
                int i = cur_end.x;
                total_cong += congestionMap2d.edge(i, j, DIR_NORTH).congestion();
                edge_num++;
                colorMap[i][j].expand = interval_index;
            }
        }
        for (int i = start.x; i <= end.x; ++i) {
            if (cur_start.y != start.y) {
                int j = cur_start.y;
                total_cong += congestionMap2d.edge(i, j, DIR_NORTH).congestion();
                edge_num++;
            }
            if (cur_end.y != end.y) {
                int j = end.y;
                total_cong += congestionMap2d.edge(i, j, DIR_NORTH).congestion();
                edge_num++;
            }
        }

        colorMap[cur_end.x][cur_end.y].expand = interval_index;

        start = cur_start;
        end = cur_end;
        avg_cong = total_cong / edge_num;
    }                                // end of while loop

    int extraExpandRange = congestion.cur_iter / 10;
    start.x = max(start.x - extraExpandRange, 0);
    end.x = min(end.x + extraExpandRange, rr_map.get_gridx() - 1);
    start.y = max(start.y - extraExpandRange, 0);
    end.y = min(end.y + extraExpandRange, rr_map.get_gridy() - 1);

    range_vector.push_back(Range_element(start.x, start.y, end.x, end.y));
}

//Rip-up the path that pass any overflowed edges, then route with monotonic 
//routing or multi-source multi-sink routing.
//If there is no overflowed path by using the two methods above, then remain 
//the original path.
void RangeRouter::range_router(Two_pin_element_2d& two_pin, int version) {
    if (!congestion.check_path_no_overflow(two_pin.path, two_pin.net_id, false)) {
        ++total_twopin;

        construct_2d_tree.NetDirtyBit[two_pin.net_id] = true;

        congestion.update_congestion_map_remove_two_pin_net(two_pin);

        std::vector<Coordinate_2d> bound_path(two_pin.path);

        Bound bound;
        bool find_path_flag = monotonicRouter.monotonicRoute(two_pin, bound, bound_path);

        if (version == 2) {
            two_pin.done = construct_2d_tree.done_iter;
        }
        if ((find_path_flag == false) || !congestion.check_path_no_overflow(bound_path, two_pin.net_id, true)) {
            Coordinate_2d start;
            Coordinate_2d end;

            start.x = min(two_pin.pin1.x, two_pin.pin2.x);
            start.y = min(two_pin.pin1.y, two_pin.pin2.y);
            end.x = max(two_pin.pin1.x, two_pin.pin2.x);
            end.y = max(two_pin.pin1.y, two_pin.pin2.y);

            int size = construct_2d_tree.BOXSIZE_INC;
            start.x = max(0, start.x - size);
            start.y = max(0, start.y - size);
            end.x = min(construct_2d_tree.rr_map.get_gridx() - 1, end.x + size);
            end.y = min(construct_2d_tree.rr_map.get_gridy() - 1, end.y + size);

            find_path_flag = construct_2d_tree.mazeroute_in_range.mm_maze_route_p(two_pin, bound.cost, bound.distance, bound.via_num, start, end, version);

            if (find_path_flag == false) {
                two_pin.path.insert(two_pin.path.begin(), bound_path.begin(), bound_path.end());
            }
        }
        construct_2d_tree.NetDirtyBit[two_pin.net_id] = true;
        congestion.update_congestion_map_insert_two_pin_net(two_pin);

    }
}

bool RangeRouter::inside_range(int left_x, int bottom_y, int right_x, int top_y, Coordinate_2d&pt) {
    if (pt.x >= left_x && pt.x <= right_x && pt.y >= bottom_y && pt.y <= top_y)
        return true;
    else
        return false;
}

void RangeRouter::query_range_2pin(int left_x, int bottom_y, int right_x, int top_y,                                //
        std::vector<Two_pin_element_2d*>& twopin_list, boost::multi_array<Point_fc, 2>& gridCell) {

    std::vector<Point_fc *> cell_list;

    static int done_counter = 0;	//only initialize once

    for (int i = left_x; i <= right_x; ++i)
        for (int j = bottom_y; j <= top_y; ++j) {
            cell_list.push_back(&(gridCell[i][j]));
        }

    for (Point_fc * cell : cell_list) {	//for each gCell
        for (Two_pin_element_2d* twopin : cell->points) {	//for each pin or steiner point
            if (twopin->done != construct_2d_tree.done_iter) {
                if (routeStateMap[twopin->pin1.x][twopin->pin1.y] != done_counter && routeStateMap[twopin->pin2.x][twopin->pin2.y] != done_counter) {
                    if (inside_range(left_x, bottom_y, right_x, top_y, twopin->pin1) || inside_range(left_x, bottom_y, right_x, top_y, (twopin->pin2))) {
                        twopin->done = construct_2d_tree.done_iter;
                        twopin_list.push_back(twopin);

                    }
                }
            }
        }
        routeStateMap[cell->x][cell->y] = done_counter;
    }
    ++done_counter;
}

void RangeRouter::specify_all_range(boost::multi_array<Point_fc, 2>& gridCell) {
    std::vector<Two_pin_element_2d *> twopin_list;
    std::vector<int> twopin_range_index_list;

    for (u_int32_t i = 0; i < colorMap.num_elements(); ++i) {
        colorMap.data()[i].set(-1, -1);
    }

    total_twopin = 0;
    for (int i = interval_list.size() - 1; i >= 0; --i) {
        Interval_element& ele = interval_list[i];
        range_vector.clear();
        sort(ele.grid_edge_vector.begin(), ele.grid_edge_vector.end(), [&](const Grid_edge_element& a, const Grid_edge_element& b) {
            return comp_grid_edge( a, b);
        });

        for (Grid_edge_element& gridEdge : ele.grid_edge_vector) {
            Coordinate_2d& c = gridEdge.grid;
            Coordinate_2d& nei = gridEdge.c2;

            if ((colorMap[c.x][c.y].expand != i) || (colorMap[nei.x][nei.y].expand != i)) {
                colorMap[c.x][c.y].expand = i;
                colorMap[nei.x][nei.y].expand = i;

                expand_range(c, nei, i);
            }
        }

        twopin_list.clear();
        twopin_range_index_list.clear();
        for (int j = 0; j < (int) range_vector.size(); ++j) {
            query_range_2pin(range_vector[j].x1, range_vector[j].y1, range_vector[j].x2, range_vector[j].y2,	//
                    twopin_list, gridCell);
        }

        sort(twopin_list.begin(), twopin_list.end(), [&](const Two_pin_element_2d *a, const Two_pin_element_2d *b) {
            return Two_pin_element_2d::comp_stn_2pin(*a,*b);});

        for (int j = 0; j < (int) twopin_list.size(); ++j) {
            range_router(*twopin_list[j], 2);
        }
    }

    twopin_list.clear();
    int length = construct_2d_tree.two_pin_list.size();
    for (int i = 0; i < length; ++i) {
        if (construct_2d_tree.two_pin_list[i].done != construct_2d_tree.done_iter) {
            twopin_list.push_back(&construct_2d_tree.two_pin_list[i]);
        }
    }

    sort(twopin_list.begin(), twopin_list.end(), [&](const Two_pin_element_2d *a, const Two_pin_element_2d *b) {
        return Two_pin_element_2d::comp_stn_2pin(*a,*b);});
    for (int i = 0; i < (int) twopin_list.size(); ++i) {
        if (twopin_list[i]->boxSize() == 1)
            break;
        range_router(*twopin_list[i], 2);
    }
    construct_2d_tree.mazeroute_in_range.clear_net_tree();
}

RangeRouter::RangeRouter(Construct_2d_tree& construct2dTree, Congestion& congestion, bool monotonic_enable) :
        total_twopin(0),	//

        construct_2d_tree { construct2dTree }, //
        congestion { congestion }, monotonicRouter { congestion, monotonic_enable } {
}
