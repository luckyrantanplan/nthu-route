/*
 * Route.h
 *
 *  Created on: Jan 16, 2018
 *      Author: florian
 */

#ifndef SRC_ROUTER_ROUTE_H_
#define SRC_ROUTER_ROUTE_H_

#include <mutex>

#include "../grdb/RoutingRegion.h"
#include "../spdlog/common.h"
#include "OutputGeneration.h"

namespace NTHUR {
class Route {
public:
    OutputGeneration process(const RoutingRegion& rr, const spdlog::level::level_enum& level = spdlog::level::warn);

private:
    std::mutex g_mutex;
};
} //end namespace NTHUR

#endif /* SRC_ROUTER_ROUTE_H_ */
