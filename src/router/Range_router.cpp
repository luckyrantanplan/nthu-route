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
    return congestion.congestionMap2d.edge(a.grid.x, a.grid.y, a.dir).congestion() > congestion.congestionMap2d.edge(b.grid.x, b.grid.y, b.dir).congestion();
}

/*
 determine INTERVAL_NUM(10) intervals between min and max,
 and also compute average congestion value (sum of demand / sum of capacity)
 */
void RangeRouter::define_interval() {

    Congestion::Statistic s = congestion.stat_congestion();

    interval_list[0].begin_value = s.max;

    for (int i = 1; i < interval_list.size(); ++i) {
        interval_list[i - 1].end_value = interval_list[i].begin_value = s.max - ((double) i / interval_list.size()) * (s.max - 1.0);
    }
    interval_list[interval_list.size() - 1].end_value = 1;
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

void RangeRouter::insert_to_interval(double cong_value, Coordinate_2d& coor_2d, int dir) {

    for (int i = interval_list.size() - 1; i >= 0; --i) {
        Interval_element& ele = interval_list[i];
        if (((cong_value < ele.begin_value) || double_equal(cong_value, ele.begin_value)) && cong_value > ele.end_value) {
            ele.grid_edge_vector.push_back(Grid_edge_element(coor_2d, dir));
            return;
        }
    }
}

void RangeRouter::divide_grid_edge_into_interval() {

    RoutingRegion& rr_map = construct_2d_tree.rr_map;
    for (int i = 0; i < congestion.congestionMap2d.getXSize() - 1; ++i) {
        for (int j = 0; j < congestion.congestionMap2d.getYSize(); ++j) {
            {
                double tmp = congestion.congestionMap2d.edge(i, j, DIR_EAST).congestion();
                if (tmp > 1)
                    insert_to_interval(tmp, construct_2d_tree.coor_array[i][j], RIGHT);
            }
            {
                double tmp = congestion.congestionMap2d.edge(i, j, DIR_NORTH).congestion();
                if (tmp > 1)
                    insert_to_interval(tmp, construct_2d_tree.coor_array[i][j], FRONT);
            }
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

void RangeRouter::expand_range(int x1, int y1, int x2, int y2, int interval_index) {
    Coordinate_2d start, end, cur_start, cur_end;
    int edge_num;
    double total_cong, avg_cong;

//(x1, y1) (x2, y2) are neighbor, so we can do the following operation
    x1 = min(x1, x2);
    x2 = max(x1, x2);
    y1 = min(y1, y2);
    y2 = max(y1, y2);
//obtain the expanded boundary. This is the very first expand, and the expand unit is 10.
    start.x = max(x1 - EXPAND_RANGE_SIZE, 0);
    RoutingRegion& rr_map = construct_2d_tree.rr_map;
    end.x = min(x2 + EXPAND_RANGE_SIZE, rr_map.get_gridx() - 1);
    start.y = max(y1 - EXPAND_RANGE_SIZE, 0);
    end.y = min(y2 + EXPAND_RANGE_SIZE, rr_map.get_gridy() - 1);

    total_cong = 0;
    edge_num = 0;
//Obtain the total congestion value and edge number of RIGHT edges.
    for (int i = start.x; i < end.x; ++i) {
        for (int j = start.y; j <= end.y; ++j) {
            total_cong += congestion.congestionMap2d.edge(i, j, DIR_EAST).congestion();
            edge_num++;
            expandMap[i][j] = interval_index;
        }
    }

//Set "visited" flag to the grid cells on the right most colum
    for (int j = start.y; j <= end.y; ++j) {
        expandMap[end.x][j] = interval_index;
    }

//Obtain the total congestion value and edge number of FRONT edges.
    for (int i = start.x; i <= end.x; ++i) {
        for (int j = start.y; j < end.y; ++j) {
            total_cong += congestion.congestionMap2d.edge(i, j, DIR_NORTH).congestion();
            edge_num++;
        }
    }

    avg_cong = total_cong / edge_num;
    while ((avg_cong > interval_list[interval_index].end_value) && (avg_cong - interval_list[interval_index].end_value > 0.01)) {
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
                total_cong += congestionMap2d.edge(i, j, DIR_EAST).congestion();
                edge_num++;
                expandMap[i][j] = interval_index;
            }
            if (cur_end.y != end.y) {
                int j = cur_end.y;
                total_cong += congestionMap2d.edge(i, j, DIR_EAST).congestion();
                edge_num++;
                expandMap[i][j] = interval_index;
            }
        }
        for (int j = start.y; j <= end.y; ++j) {
            if (cur_start.x != start.x) {
                int i = cur_start.x;
                total_cong += congestionMap2d.edge(i, j, DIR_EAST).congestion();
                edge_num++;
            }
            if (cur_end.x != end.x) {
                int i = end.x;
                total_cong += congestionMap2d.edge(i, j, DIR_EAST).congestion();
                edge_num++;
            }
        }
        for (int j = cur_start.y; j < cur_end.y; ++j) {
            if (cur_start.x != start.x) {
                int i = cur_start.x;
                total_cong += congestionMap2d.edge(i, j, DIR_NORTH).congestion();
                edge_num++;
                expandMap[i][j] = interval_index;
            }
            if (cur_end.x != end.x) {
                int i = cur_end.x;
                total_cong += congestionMap2d.edge(i, j, DIR_NORTH).congestion();
                edge_num++;
                expandMap[i][j] = interval_index;
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

        expandMap[cur_end.x][cur_end.y] = interval_index;

        start = cur_start;
        end = cur_end;
        avg_cong = total_cong / edge_num;
    }                                // end of while loop

    int extraExpandRange = construct_2d_tree.cur_iter / 10;
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
void RangeRouter::range_router(Two_pin_element_2d& two_pin) {
    if (!congestion.check_path_no_overflow(two_pin.path, two_pin.net_id, false)) {
        ++total_twopin;

        construct_2d_tree.NetDirtyBit[two_pin.net_id] = true;
        construct_2d_tree.update_congestion_map_remove_two_pin_net(two_pin);

        std::vector<Coordinate_2d*> bound_path(two_pin.path);

        Monotonic_element mn;
        construct_2d_tree.post_processing.compute_path_total_cost_and_distance(two_pin, mn);
        double bound_cost = mn.total_cost;
        int bound_distance = mn.distance;
        int bound_via_num = mn.via_num;

        two_pin.path.clear();

        bool find_path_flag = false;

        if (construct_2d_tree.routing_parameter.get_monotonic_en()) {
            bool find_path_flag = construct_2d_tree.monotonic_pattern_route(two_pin.pin1.x, two_pin.pin1.y, two_pin.pin2.x, two_pin.pin2.y, two_pin, two_pin.net_id, bound_cost, bound_distance,
                    bound_via_num, true);

            if (find_path_flag) {
                delete bound_path;
                bound_path = new std::vector<Coordinate_2d*>(two_pin.path);
                bound_cost = construct_2d_tree.cong_monotonic[two_pin.path[0]->x][two_pin.path[0]->y].total_cost;
                bound_distance = construct_2d_tree.cong_monotonic[two_pin.path[0]->x][two_pin.path[0]->y].distance;
                bound_via_num = construct_2d_tree.cong_monotonic[two_pin.path[0]->x][two_pin.path[0]->y].via_num;
            }
        }

        two_pin.done = construct_2d_tree.done_iter;

        if ((find_path_flag == false) || !construct_2d_tree.post_processing.check_path_no_overflow(bound_path, two_pin.net_id, true)) {
            Coordinate_2d start, end;

            start.x = min(two_pin.pin1.x, two_pin.pin2.x);
            end.x = max(two_pin.pin1.x, two_pin.pin2.x);
            start.y = min(two_pin.pin1.y, two_pin.pin2.y);
            end.y = max(two_pin.pin1.y, two_pin.pin2.y);

            int size = construct_2d_tree.BOXSIZE_INC;
            start.x = max(0, start.x - size);
            start.y = max(0, start.y - size);
            end.x = min(construct_2d_tree.rr_map.get_gridx() - 1, end.x + size);
            end.y = min(construct_2d_tree.rr_map.get_gridy() - 1, end.y + size);

            find_path_flag = construct_2d_tree.mazeroute_in_range->mm_maze_route_p2(two_pin, bound_cost, bound_distance, bound_via_num, start, end);

            if (find_path_flag == false) {
                two_pin.path.insert(two_pin.path.begin(), bound_path->begin(), bound_path->end());
            }
        }

        construct_2d_tree.update_congestion_map_insert_two_pin_net(two_pin);
        delete (bound_path);
    }
}

bool RangeRouter::inside_range(int left_x, int bottom_y, int right_x, int top_y, Coordinate_2d *pt) {
    if (pt->x >= left_x && pt->x <= right_x && pt->y >= bottom_y && pt->y <= top_y)
        return true;
    else
        return false;
}

void RangeRouter::query_range_2pin(int left_x, int bottom_y, int right_x, int top_y,                                //
        std::vector<Two_pin_element_2d *>& twopin_list, boost::multi_array<Point_fc, 2>& gridCell) {

    std::vector<Point_fc *> cell_list;
    int len;
    static int done_counter = 0;	//only initialize once
    int query_twopin_num = 0; // added by grey

    for (int i = left_x; i <= right_x; ++i)
        for (int j = bottom_y; j <= top_y; ++j) {
            cell_list.push_back(&(gridCell.vertex(i, j)));
        }

    len = cell_list.size();
    for (int i = 0; i < len; ++i)	//for each gCell
            {
        for (std::vector<Two_pin_element_2d*>::iterator it = cell_list[i]->points.begin(); it != cell_list[i]->points.end(); ++it)	//for each pin or steiner point
                {
            if ((*it)->done != construct_2d_tree.done_iter) {
                if (routeStateMap->color((*it)->pin1.x, (*it)->pin1.y) != done_counter && routeStateMap->color((*it)->pin2.x, (*it)->pin2.y) != done_counter) {
                    if (inside_range(left_x, bottom_y, right_x, top_y, &((*it)->pin1)) || inside_range(left_x, bottom_y, right_x, top_y, &((*it)->pin2))) {
                        (*it)->done = construct_2d_tree.done_iter;
                        twopin_list->push_back(*it);
                        query_twopin_num++;
                    }
                }
            }
        }
        routeStateMap->color(cell_list[i]->x, cell_list[i]->y) = done_counter;
    }
    ++done_counter;
}

void RangeRouter::specify_all_range(boost::multi_array<Point_fc, 2>& gridCell) {
    std::vector<Two_pin_element_2d *> twopin_list;
    std::vector<int> twopin_range_index_list;

    int x = construct_2d_tree.rr_map.get_gridx();
    int y = construct_2d_tree.rr_map.get_gridy();

    expandMap(x, y, -1);
    routeStateMap(x, y, -1);

    total_twopin = 0;
    for (int i = intervalCount - 1; i >= 0; --i) {

        range_vector.clear();
        sort(interval_list[i].grid_edge_vector.begin(), interval_list[i].grid_edge_vector.end(), [&](const Grid_edge_element* a, const Grid_edge_element* b) {
            return comp_grid_edge(a,b);
        });

        for (int j = 0; j < (int) interval_list[i].grid_edge_vector.size(); ++j) {
            int x = interval_list[i].grid_edge_vector[j].grid.x;
            int y = interval_list[i].grid_edge_vector[j].grid.y;
            int dir = interval_list[i].grid_edge_vector[j].dir;

            int nei_x = x;
            int nei_y = y;
            if (dir == RIGHT)
                ++nei_x;
            else
                ++nei_y;

            if ((expandMap[x][y] != i) || (expandMap[nei_x][nei_y] != i)) {
                expandMap[x][y] = i;
                expandMap[nei_x][nei_y] = i;

                expand_range(x, y, nei_x, nei_y, i);
            }
        }

        twopin_list.clear();
        twopin_range_index_list.clear();
        for (int j = 0; j < (int) range_vector.size(); ++j) {
            query_range_2pin(range_vector[j]->x1, range_vector[j]->y1, range_vector[j]->x2, range_vector[j]->y2,	//
                    &twopin_list, gridCell);
        }

        sort(twopin_list.begin(), twopin_list.end(), [&](const Two_pin_element_2d *a, const Two_pin_element_2d *b) {
            return construct_2d_tree.comp_stn_2pin(a,b);});

        for (int j = 0; j < (int) twopin_list.size(); ++j) {
            range_router(twopin_list[j]);
        }
    }

    twopin_list.clear();
    int length = construct_2d_tree.two_pin_list.size();
    for (int i = 0; i < length; ++i) {
        if (construct_2d_tree.two_pin_list[i]->done != construct_2d_tree.done_iter) {
            twopin_list.push_back(construct_2d_tree.two_pin_list[i]);
        }
    }

    sort(twopin_list.begin(), twopin_list.end(), [&](const Two_pin_element_2d *a, const Two_pin_element_2d *b) {
        return construct_2d_tree.comp_stn_2pin(a,b);});
    for (int i = 0; i < (int) twopin_list.size(); ++i) {
        if (twopin_list[i]->boxSize() == 1)
            break;
        range_router(twopin_list[i]);
    }
    construct_2d_tree.mazeroute_in_range->clear_net_tree();
}

RangeRouter::RangeRouter(Construct_2d_tree& construct2dTree, Congestion& congestion) :
        total_twopin(0), num_of_grid_edge(0), min_congestion(0.), max_congestion(0.), avg_congestion(0.), intervalCount { INTERVAL_NUM }, construct_2d_tree { construct2dTree }, congestion { congestion } {
}
