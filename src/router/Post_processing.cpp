#include "Construct_2d_tree.h"
#include "Post_processing.h"
#include "../misc/geometry.h"
#include <algorithm>

using namespace Jm;

#include "MM_mazeroute.h"
#include "Route_2pinnets.h"
int cur_overflow;
int pre_overflow;

static int Post_processing_iteration;
static int inc_num;
static bool total_no_overflow;

int Post_processing::comp(const COUNTER& a, const COUNTER& b) {
    if (a.total_overflow > b.total_overflow)
        return true;
    else if (a.total_overflow < b.total_overflow)
        return false;
    else if (a.bsize > b.bsize)
        return true;
    else
        return false;
}

//Check if the specified edge is not overflowed
//Return false if the edge is overflowed
bool Post_processing::check_path_no_overflow(vector<Coordinate_2d*> *path, int net_id, int inc_flag) {
    for (int i = path->size() - 2; i >= 0; --i) {
        int dir = get_direction_2d((*path)[i], (*path)[i + 1]);
        //There are two modes:
        // 1. inc_flag = 0: Just report if the specified edge is overflowd
        // 2. inc_flag = 1: Check if the specified edge will be overflowed if wd add a demond on it.
        if (inc_flag == 0) {
            if (congestionMap2d->edge((*path)[i]->x, (*path)[i]->y, dir).isOverflow())
                return false;
        } else {
            int inc = 1;
            //If there is another 2-pin net from the same net is using the specified edge,
            //then we don't need to increase a demand on it. We can use the edge directly
            if (congestionMap2d->edge((*path)[i]->x, (*path)[i]->y, dir).lookupNet(net_id))
                inc = 0;

            if (congestionMap2d->edge((*path)[i]->x, (*path)[i]->y, dir).cur_cap + inc > congestionMap2d->edge((*path)[i]->x, (*path)[i]->y, dir).max_cap)
                return false;
        }
    }
    return true;
}

//Obtain a cost of a path, including via cost.
void Post_processing::compute_path_total_cost_and_distance(Two_pin_element_2d *element, Monotonic_element* mn) {
    int distance;
    int pre_dir = -1;

    mn->total_cost = 0;
    mn->distance = 0;
    mn->via_num = 0;
    for (int i = element->path.size() - 2; i >= 0; --i) {
        int dir = get_direction_2d(element->path[i], element->path[i + 1]);
        mn->total_cost += get_cost_2d(element->path[i]->x, element->path[i]->y, dir, element->net_id, &distance);
        mn->distance += distance;
        if (pre_dir != -1) {
            //if the wire need to bend, then we need to add via cost to it
            if ((pre_dir < 2 && dir >= 2) || (pre_dir >= 2 && dir < 2)) {
                mn->via_num += via_cost;
                if (used_cost_flag == HISTORY_COST) {
                    mn->total_cost += via_cost;
                }
            }
        }
        pre_dir = dir;
    }
}

void Post_processing::initial_for_post_processing() {
    int i, j, edge_idx, x_dir, y_dir, x, y, id;
    vector<COUNTER> counter(two_pin_list.size());
    bool no_overflow;
    Two_pin_list_2d temp_list;
    Multisource_multisink_mazeroute* mazeroute = mazeroute_in_range;
    double bound_cost;
    int bound_distance, bound_via_num;
    bool find_path_flag = false;
    vector<Coordinate_2d*> *bound_path;

    for (i = two_pin_list.size() - 1; i >= 0; --i) {
        if (two_pin_list[i]->path.size() == 0)
            temp_list.push_back(two_pin_list[i]);
        counter[i].id = i;
        counter[i].total_overflow = 0;
        counter[i].bsize = abs(two_pin_list[i]->pin1.x - two_pin_list[i]->pin2.x) + abs(two_pin_list[i]->pin1.y - two_pin_list[i]->pin2.y);
        for (j = two_pin_list[i]->path.size() - 1; j > 0; --j) {
            x_dir = two_pin_list[i]->path[j - 1]->x - two_pin_list[i]->path[j]->x;
            y_dir = two_pin_list[i]->path[j - 1]->y - two_pin_list[i]->path[j]->y;
            if (x_dir)
                edge_idx = (x_dir == 1) ? RIGHT : LEFT;
            else
                // y_dir
                edge_idx = (y_dir == 1) ? FRONT : BACK;
            x = two_pin_list[i]->path[j]->x;
            y = two_pin_list[i]->path[j]->y;
            if (congestionMap2d->edge(x, y, edge_idx).isOverflow()) {
                counter[i].total_overflow += max(0, congestionMap2d->edge(x, y, edge_idx).overUsage());
            }
        }
        if (counter[i].total_overflow > 0) {
            total_no_overflow = false;
        }
    }

    if (total_no_overflow)
        return;

    sort(counter.begin(), counter.end(), comp);	// sort by flag

    Monotonic_element mn;
    // acording other attribute to do maze routing
    for (i = 0; i < (int) two_pin_list.size(); ++i) {
        id = counter[i].id;
        // call maze routing
        if (counter[i].total_overflow > 0) {

            no_overflow = check_path_no_overflow(&two_pin_list[id]->path, two_pin_list[id]->net_id, false);
            if (no_overflow) {
                continue;
            }

            NetDirtyBit[two_pin_list[id]->net_id] = true;
            update_congestion_map_remove_two_pin_net (two_pin_list[id]);

            bound_path = new vector<Coordinate_2d*>(two_pin_list[id]->path);
            compute_path_total_cost_and_distance(two_pin_list[id], &mn);
            bound_cost = mn.total_cost;
            bound_distance = mn.distance;
            bound_via_num = mn.via_num;
            two_pin_list[id]->path.clear();

            if (routing_parameter->get_monotonic_en()) {
                find_path_flag = monotonic_pattern_route(two_pin_list[id]->pin1.x, two_pin_list[id]->pin1.y, two_pin_list[id]->pin2.x, two_pin_list[id]->pin2.y, two_pin_list[id],
                        two_pin_list[id]->net_id, bound_cost, bound_distance, bound_via_num, true);
                if (find_path_flag) {
                    delete bound_path;
                    bound_path = new vector<Coordinate_2d*>(two_pin_list[id]->path);
                    bound_cost = cong_monotonic[two_pin_list[id]->path[0]->x][two_pin_list[id]->path[0]->y].total_cost;
                    bound_distance = cong_monotonic[two_pin_list[id]->path[0]->x][two_pin_list[id]->path[0]->y].distance;
                    bound_via_num = cong_monotonic[two_pin_list[id]->path[0]->x][two_pin_list[id]->path[0]->y].via_num;
                }
            }

            if (find_path_flag && check_path_no_overflow(bound_path, two_pin_list[id]->net_id, true)) {
            } else {
                Coordinate_2d start, end;
                start.x = min(two_pin_list[id]->pin1.x, two_pin_list[id]->pin2.x);
                end.x = max(two_pin_list[id]->pin1.x, two_pin_list[id]->pin2.x);
                start.y = min(two_pin_list[id]->pin1.y, two_pin_list[id]->pin2.y);
                end.y = max(two_pin_list[id]->pin1.y, two_pin_list[id]->pin2.y);
                start.x = max(0, start.x - BOXSIZE_INC);
                start.y = max(0, start.y - BOXSIZE_INC);
                end.x = min(rr_map->get_gridx() - 1, end.x + BOXSIZE_INC);
                end.y = min(rr_map->get_gridy() - 1, end.y + BOXSIZE_INC);
                find_path_flag = mazeroute->mm_maze_route_p3(two_pin_list[id], bound_cost, bound_distance, bound_via_num, start, end);
                if (find_path_flag == false) {
                    two_pin_list[id]->path.clear();
                    two_pin_list[id]->path.insert(two_pin_list[id]->path.begin(), bound_path->begin(), bound_path->end());
                }
            }
            update_congestion_map_insert_two_pin_net(two_pin_list[id]);
            delete (bound_path);
        }
    }
    mazeroute->clear_net_tree();
}

Post_processing::Post_processing(Construct_2d_tree& construct_2d_tree) :
        construct_2d_tree { construct_2d_tree } {
    cur_overflow = -1;

    //Fetch from routing_parameter 
    RoutingParameters* routing_parameter = construct_2d_tree.routing_parameter;

    routing_parameter->BOXSIZE_INC = routing_parameter->get_init_box_size_p3();
    inc_num = routing_parameter->get_box_size_inc_p3();
    Post_processing_iteration = routing_parameter->get_iteration_p3();
#ifdef MESSAGE
    printf("size: (%d %d)\n",BOXSIZE_INC,inc_num);
#endif
    construct_2d_tree.done_iter++;
    construct_2d_tree.used_cost_flag = MADEOF_COST;
    cur_overflow = construct_2d_tree.cal_max_overflow();
    if (cur_overflow > 0) {
        //In post processing, we only need to pre-evaluate all cost once.
        //The other update will be done by update_add(remove)_edge
        pre_evaluate_congestion_cost();
        for (int i = 0; i < Post_processing_iteration; ++i, ++done_iter) {
            printf("\033[31mIteration: \033[m%d\n", i + 1);
#ifdef MESSAGE
            monotonic_solved_counter = maze_solved_counter = no_overflow_counter = 0;
            printf("no_overflow:%d monotonic:%d maze:%d\n",
                    no_overflow_counter,monotonic_solved_counter,maze_solved_counter);
            printf("pre_of:%d cur_of:%d\n",pre_overflow,cur_overflow);
#endif
            total_no_overflow = true;

            initial_for_post_processing();

            pre_overflow = cur_overflow;
            cur_overflow = cal_max_overflow();
            cal_total_wirelength();

            if (total_no_overflow || cur_overflow == 0)
                break;
            BOXSIZE_INC += inc_num;
            reallocate_two_pin_list(true);
        }
    }

    for (int i = 0; i < (int) two_pin_list.size(); ++i) {
        insert_all_two_pin_list (two_pin_list[i]);
    }

    delete cache;
#ifdef MESSAGE
    puts("maze routing complete successfully");
#endif
    init_3d_map();	        //allocate space
    output_3d_map();	    //assign max_cap
}
