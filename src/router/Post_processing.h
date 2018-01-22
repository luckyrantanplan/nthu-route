#ifndef _Post_processing_H_
#define _Post_processing_H_

#include <memory>

namespace spdlog {
class logger;
} /* namespace spdlog */

namespace NTHUR {
class RoutingParameters;
class Congestion;
struct RangeRouter;
struct Route_2pinnets;

struct COUNTER {
    int id;
    double total_overflow;
    int bsize;

    bool operator <(const COUNTER& o) const;

};

class Construct_2d_tree;
class Two_pin_element_2d;

struct Post_processing {
    const RoutingParameters& routing_parameter;
    Congestion& congestion;

    bool total_no_overflow;

    Construct_2d_tree& construct_2d_tree;
    RangeRouter& rangeRouter;
    std::shared_ptr<spdlog::logger> log_sp;
    void initial_for_post_processing();
    Post_processing(const RoutingParameters& routingparam, Congestion& congestion, Construct_2d_tree& construct_2d_tree, RangeRouter& rangeRouter);
    void process(Route_2pinnets& route_2pinnets);
};
} // namespace NTHUR

#endif
