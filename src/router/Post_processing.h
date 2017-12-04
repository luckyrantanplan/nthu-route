#ifndef _Post_processing_H_
#define _Post_processing_H_
class Congestion;
struct RangeRouter;
struct Route_2pinnets;

struct COUNTER {
    int id;
    double total_overflow;
    int bsize;
};

class Construct_2d_tree;
class Two_pin_element_2d;
class Monotonic_element;

struct Post_processing {
    Congestion& congestion;
    int cur_overflow;
    int pre_overflow;

    int Post_processing_iteration;
    int inc_num;
    bool total_no_overflow;

    Construct_2d_tree& construct_2d_tree;
    RangeRouter& rangeRouter;
    int comp(const COUNTER& a, const COUNTER& b);

    void initial_for_post_processing();
    Post_processing(Congestion& congestion, Construct_2d_tree& construct_2d_tree, RangeRouter& rangeRouter);
    void process(Route_2pinnets& route_2pinnets);
};
#endif
