#ifndef _Post_processing_H_
#define _Post_processing_H_

#include <vector>

#include "../misc/geometry.h"

struct Route_2pinnets;

#define error_bound 0.00000000001
#define neg_error_bound -0.00000000001

struct COUNTER {
    int id;
    double total_overflow;
    int bsize;
};

class Construct_2d_tree;
class Two_pin_element_2d;
class Monotonic_element;

struct Post_processing {

    int cur_overflow;
    int pre_overflow;

    static int Post_processing_iteration;
    static int inc_num;
    static bool total_no_overflow;

    Construct_2d_tree& construct_2d_tree;

    int comp(const COUNTER& a, const COUNTER& b);

    bool check_path_no_overflow(std::vector<Jm::Coordinate_2d*> *path, int net_id, int inc_flag);

    void compute_path_total_cost_and_distance(Two_pin_element_2d *element, Monotonic_element*);
    void initial_for_post_processing();
    Post_processing(Construct_2d_tree& construct_2d_tree);
    void process(Route_2pinnets& route_2pinnets);
};
#endif
