#include "Construct_2d_tree.h"
#include "Post_processing.h"
#include "../misc/geometry.h"
#include <algorithm>

#include "MM_mazeroute.h"
#include "Route_2pinnets.h"
#include "../grdb/RoutingRegion.h"
#include "parameter.h"

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



void Post_processing::initial_for_post_processing() {
    int i, j, edge_idx, x_dir, y_dir, x, y, id;
    vector<COUNTER> counter(construct_2d_tree.two_pin_list.size());
    bool no_overflow;

    Multisource_multisink_mazeroute* mazeroute = construct_2d_tree.mazeroute_in_range;
    double bound_cost;
    int bound_distance, bound_via_num;
    bool find_path_flag = false;
    vector<Coordinate_2d*> *bound_path;

    for (i = construct_2d_tree.two_pin_list.size() - 1; i >= 0; --i) {

        Two_pin_element_2d& twopList = construct_2d_tree.two_pin_list[i];

        counter[i].id = i;
        counter[i].total_overflow = 0;
        counter[i].bsize = abs(twopList.pin1.x - twopList.pin2.x) + abs(twopList.pin1.y - twopList.pin2.y);
        for (j = twopList.path.size() - 1; j > 0; --j) {
            x_dir = twopList.path[j - 1].x - twopList.path[j].x;
            y_dir = twopList.path[j - 1].y - twopList.path[j].y;
            if (x_dir)
                edge_idx = (x_dir == 1) ? RIGHT : LEFT;
            else
                // y_dir
                edge_idx = (y_dir == 1) ? FRONT : BACK;
            x = twopList.path[j].x;
            y = twopList.path[j].y;
            if (construct_2d_tree.congestionMap2d.edge(x, y, edge_idx).isOverflow()) {
                counter[i].total_overflow += max(0, construct_2d_tree.congestionMap2d.edge(x, y, edge_idx).overUsage());
            }
        }
        if (counter[i].total_overflow > 0) {
            total_no_overflow = false;
        }
    }

    if (total_no_overflow) {
        return;
    }

    std::sort(counter.begin(), counter.end(), [&](COUNTER& a,COUNTER& b ) {return comp(a,b);});	// sort by flag


    // According other attribute to do maze routing
    for (i = 0; i < (int) construct_2d_tree.two_pin_list.size(); ++i) {
        id = counter[i].id;
        Two_pin_element_2d& twopList = construct_2d_tree.two_pin_list[id];
        // call maze routing
        if (counter[i].total_overflow > 0) {

            no_overflow = check_path_no_overflow(twopList.path, twopList.net_id, false);
            if (no_overflow) {
                continue;
            }

            construct_2d_tree.NetDirtyBit[twopList.net_id] = true;
            construct_2d_tree.update_congestion_map_remove_two_pin_net(twopList);

            bound_path = new vector<Coordinate_2d*>(twopList.path);

            Monotonic_element mn;
            compute_path_total_cost_and_distance(twopList, &mn);
            bound_cost = mn.total_cost;
            bound_distance = mn.distance;
            bound_via_num = mn.via_num;

            twopList.path.clear();

            if (construct_2d_tree.routing_parameter.get_monotonic_en()) {
                find_path_flag = construct_2d_tree.monotonic_pattern_route(twopList.pin1.x, twopList.pin1.y, twopList.pin2.x, twopList.pin2.y,	//
                        twopList, twopList.net_id, bound_cost, bound_distance, bound_via_num, true);
                if (find_path_flag) {
                    delete bound_path;
                    bound_path = new vector<Coordinate_2d*>(twopList.path);
                    bound_cost = construct_2d_tree.cong_monotonic[twopList.path[0].x][twopList.path[0].y].total_cost;
                    bound_distance = construct_2d_tree.cong_monotonic[twopList.path[0].x][twopList.path[0].y].distance;
                    bound_via_num = construct_2d_tree.cong_monotonic[twopList.path[0].x][twopList.path[0].y].via_num;
                }
            }

            if (find_path_flag && check_path_no_overflow(bound_path, twopList.net_id, true)) {
            } else {
                Coordinate_2d start, end;
                start.x = min(twopList.pin1.x, twopList.pin2.x);
                end.x = max(twopList.pin1.x, twopList.pin2.x);
                start.y = min(twopList.pin1.y, twopList.pin2.y);
                end.y = max(twopList.pin1.y, twopList.pin2.y);
                int BOXSIZE_INC = construct_2d_tree.routing_parameter.BOXSIZE_INC;
                start.x = max(0, start.x - BOXSIZE_INC);
                start.y = max(0, start.y - BOXSIZE_INC);
                end.x = min(construct_2d_tree.rr_map.get_gridx() - 1, end.x + BOXSIZE_INC);
                end.y = min(construct_2d_tree.rr_map.get_gridy() - 1, end.y + BOXSIZE_INC);
                find_path_flag = mazeroute->mm_maze_route_p3(twopList, bound_cost, bound_distance, bound_via_num, start, end);
                if (find_path_flag == false) {
                    twopList.path.clear();
                    twopList.path.insert(twopList.path.begin(), bound_path->begin(), bound_path->end());
                }
            }
            construct_2d_tree.update_congestion_map_insert_two_pin_net(twopList);
            delete (bound_path);
        }
    }
    mazeroute->clear_net_tree();
}

Post_processing::Post_processing(Construct_2d_tree& construct_2d_tree) :
        construct_2d_tree { construct_2d_tree } {
}

void Post_processing::process(Route_2pinnets& route_2pinnets) {
    cur_overflow = -1;

    //Fetch from routing_parameter 
    RoutingParameters& routing_parameter = construct_2d_tree.routing_parameter;

    routing_parameter.BOXSIZE_INC = routing_parameter.get_init_box_size_p3();
    inc_num = routing_parameter.get_box_size_inc_p3();
    Post_processing_iteration = routing_parameter.get_iteration_p3();
#ifdef MESSAGE
    printf("size: (%d %d)\n",BOXSIZE_INC,inc_num);
#endif
    construct_2d_tree.done_iter++;
    construct_2d_tree.used_cost_flag = MADEOF_COST;
    cur_overflow = construct_2d_tree.cal_max_overflow();
    if (cur_overflow > 0) {
        //In post processing, we only need to pre-evaluate all cost once.
        //The other update will be done by update_add(remove)_edge
        construct_2d_tree.pre_evaluate_congestion_cost();
        for (int i = 0; i < Post_processing_iteration; ++i, ++construct_2d_tree.done_iter) {
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
            cur_overflow = construct_2d_tree.cal_max_overflow();
            construct_2d_tree.cal_total_wirelength();

            if (total_no_overflow || cur_overflow == 0)
                break;
            construct_2d_tree.BOXSIZE_INC += inc_num;
            route_2pinnets.reallocate_two_pin_list(true);
        }
    }

    for (int i = 0; i < (int) construct_2d_tree.two_pin_list.size(); ++i) {
        construct_2d_tree.insert_all_two_pin_list(construct_2d_tree.two_pin_list[i]);
    }

    delete construct_2d_tree.cache;
#ifdef MESSAGE
    puts("maze routing complete successfully");
#endif
    construct_2d_tree.init_3d_map();	        //allocate space
    construct_2d_tree.output_3d_map();	    //assign max_cap
}
