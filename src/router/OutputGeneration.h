/*
 * OutputGeneration.h
 *
 *  Created on: Jan 15, 2018
 *      Author: florian
 */

#ifndef SRC_ROUTER_OUTPUTGENERATION_H_
#define SRC_ROUTER_OUTPUTGENERATION_H_

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "../grdb/EdgePlane3d.h"
#include "../grdb/RoutingComponent.h"
#include "../grdb/RoutingRegion.h"
#include "../misc/geometry.h"

namespace spdlog {
class logger;
} /* namespace spdlog */

namespace NTHUR {
class RoutingRegion;
} /* namespace NTHUR */

namespace NTHUR {

class Edge_3d {
public:
    Edge_3d() :
            max_cap(0), cur_cap(0), used_net(5) {

    }
public:
    typedef std::unordered_map<int, int> LRoutedNetTable;
    bool isOverflow() const {
        return (cur_cap > max_cap);
    }
    int overUsage() const {
        return (cur_cap - max_cap);
    }
    int max_cap;
    int cur_cap;
    LRoutedNetTable used_net;

    std::string toString() const {
        std::string s = "max_cap: " + std::to_string(max_cap);
        s += " cur_cap: " + std::to_string(cur_cap);
        s += " used_net: " + std::to_string(used_net.size());
        return s;
    }

};

class OutputGeneration {

public:
    typedef std::vector<std::vector<Segment3d> > Comb;

    OutputGeneration(const RoutingRegion& rr);

    void generate_output(const int net_id, const std::vector<Segment3d>& v, std::ostream & output) const;
    void collectComb(Coordinate_3d c2, Coordinate_3d& c, Comb& comb) const;
    void plotNet(int net_id) const;
    void printEdge(const Coordinate_3d& c, const Coordinate_3d& c2) const;
    void generate_all_output(std::ostream & output) const;
    void print_max_overflow() const;
    void calculate_wirelength(const int global_via_cost) const;

    const std::vector<Net::Pin>& get_nPin(int net_id) const;
    std::size_t get_netNumber() const;

    EdgePlane3d<Edge_3d> cur_map_3d;
    Comb combAllNet() const;
private:
    void scale(Comb& comb) const;
    const RoutingRegion& rr_map;
    std::shared_ptr<spdlog::logger> log_sp;

}
;

inline const std::vector<Net::Pin>& OutputGeneration::get_nPin(int net_id) const {
    //get Pins by net
    return rr_map.get_net(net_id).get_pinList();
}

inline std::size_t OutputGeneration::get_netNumber() const {
    return rr_map.get_netNumber();
}
} /* namespace NTHUR */

#endif /* SRC_ROUTER_OUTPUTGENERATION_H_ */
