/*
 * Bom.h
 *
 *  Created on: Nov 29, 2017
 *      Author: florian
 */

#ifndef SRC_ROUTER_CONGESTION_H_
#define SRC_ROUTER_CONGESTION_H_

#include <functional>
#include <memory>
#include <vector>

#include "../grdb/EdgePlane.h"
#include "../misc/geometry.h"
#include "DataDef.h"

namespace spdlog {
class logger;
} /* namespace spdlog */
namespace NTHUR {
class RoutingRegion;

class Congestion {
public:

    struct Statistic {

        double min;
        double max;
        double avg;
    };

    std::function<void(Edge_2d& edge)> pre_evaluate_congestion_cost_fp;

    int via_cost;
    int used_cost_flag;
    double exponent;
    double WL_Cost;
    double factor;
    int cur_iter;
    EdgePlane<Edge_2d> congestionMap2d;
    std::shared_ptr<spdlog::logger> log_sp;
    Congestion(int x, int y);

    double get_cost_2d(const Coordinate_2d& c1, const Coordinate_2d& c2, int net_id, int& distance);
    int cal_max_overflow();
    void pre_evaluate_congestion_cost_all(Edge_2d& edge) const;
    void pre_evaluate_congestion_cost();
    bool check_path_no_overflow(const std::vector<Coordinate_2d>&path, const int net_id, const int inc_flag) const;
    int find_overflow_max(int max_zz) const;
    void init_2d_map(const RoutingRegion& rr_map);
    int cal_total_wirelength() const;
    Statistic stat_congestion();
    void update_congestion_map_insert_two_pin_net(Two_pin_element_2d& element);
    void update_congestion_map_remove_two_pin_net(const std::vector<Coordinate_2d>& path, const int net_id);
    void calculate_cap() const;
    std::string plotCongestionNet(int net_id) const;
};

} // namespace NTHUR

#endif /* SRC_ROUTER_CONGESTION_H_ */
