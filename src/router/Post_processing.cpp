#include "Post_processing.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "../misc/geometry.h"
#include "Congestion.h"
#include "Construct_2d_tree.h"
#include "DataDef.h"
#include "parameter.h"
#include "Range_router.h"
#include "Route_2pinnets.h"

bool COUNTER::operator <(const COUNTER& o) const {
    return std::tie(total_overflow, bsize) < std::tie(o.total_overflow, o.bsize);

}

void Post_processing::initial_for_post_processing() {

    vector<COUNTER> counter(construct_2d_tree.two_pin_list.size());

    for (int i = construct_2d_tree.two_pin_list.size() - 1; i >= 0; --i) {

        Two_pin_element_2d& twopList = construct_2d_tree.two_pin_list[i];

        counter[i].id = i;
        counter[i].total_overflow = 0;
        counter[i].bsize = abs(twopList.pin1.x - twopList.pin2.x) + abs(twopList.pin1.y - twopList.pin2.y);
        for (int j = twopList.path.size() - 1; j > 0; --j) {

            Edge_2d& edge = congestion.congestionMap2d.edge(twopList.path[j - 1], twopList.path[j]);
            if (edge.isOverflow()) {
                counter[i].total_overflow += max(0, edge.overUsage());
            }
        }
        if (counter[i].total_overflow > 0) {
            total_no_overflow = false;
        }
    }

    if (total_no_overflow) {
        return;
    }

    std::sort(counter.begin(), counter.end(), [&](COUNTER& a,COUNTER& b ) {return b< a;});	// sort by flag

    // According other attribute to do maze routing
    for (int i = 0; i < (int) construct_2d_tree.two_pin_list.size(); ++i) {
        int id = counter[i].id;
        Two_pin_element_2d& twopList = construct_2d_tree.two_pin_list[id];
        // call maze routing
        if (counter[i].total_overflow > 0) {
            rangeRouter.range_router(twopList, 3);
        }
    }

}

Post_processing::Post_processing(Congestion& congestion, Construct_2d_tree& construct_2d_tree, RangeRouter& rangeRouter) :
        congestion { congestion },	//
        total_no_overflow { false },	//
        construct_2d_tree { construct_2d_tree }, //
        rangeRouter { rangeRouter } {
}

void Post_processing::process(Route_2pinnets& route_2pinnets) {

    //Fetch from routing_parameter 
    RoutingParameters& routing_parameter = construct_2d_tree.routing_parameter;

    routing_parameter.BOXSIZE_INC = routing_parameter.get_init_box_size_p3();
    int inc_num = routing_parameter.get_box_size_inc_p3();
    Post_processing_iteration = routing_parameter.get_iteration_p3();
#ifdef MESSAGE
    printf("size: (%d %d)\n",BOXSIZE_INC,inc_num);
#endif
    construct_2d_tree.done_iter++;
    congestion.used_cost_flag = MADEOF_COST;
    int cur_overflow = congestion.cal_max_overflow();
    if (cur_overflow > 0) {
        //In post processing, we only need to pre-evaluate all cost once.
        //The other update will be done by update_add(remove)_edge
        congestion.pre_evaluate_congestion_cost();
        for (int i = 0; i < Post_processing_iteration; ++i, ++construct_2d_tree.done_iter) {
            printf("\033[31mIteration: \033[m%d\n", i + 1);
#ifdef MESSAGE
            monotonic_solved_counter = maze_solved_counter = no_overflow_counter = 0;
            printf("no_overflow:%d monotonic:%d maze:%d\n",
                    no_overflow_counter,monotonic_solved_counter,maze_solved_counter);
            printf("pre_of:%d cur_of:%d\n", cur_overflow);
#endif
            total_no_overflow = true;

            initial_for_post_processing();

            cur_overflow = congestion.cal_max_overflow();
            congestion.cal_total_wirelength();

            if (total_no_overflow || cur_overflow == 0)
                break;
            construct_2d_tree.BOXSIZE_INC += inc_num;
            route_2pinnets.reallocate_two_pin_list();
        }
    }

#ifdef MESSAGE
    puts("maze routing complete successfully");
#endif

}
