/*
 * Route.h
 *
 *  Created on: Jan 16, 2018
 *      Author: florian
 */

#ifndef SRC_ROUTER_ROUTE_H_
#define SRC_ROUTER_ROUTE_H_

#include "Congestion.h"
#include "Construct_2d_tree.h"
#include "Layerassignment.h"
#include "OutputGeneration.h"

namespace NTHUR {
class Route {

    OutputGeneration process(const RoutingRegion& rr) {
        Congestion congestion(rr.get_gridx(), rr.get_gridy());
        RoutingParameters routingparam; // default settings

        Construct_2d_tree tree(routingparam, rr, congestion);

        OutputGeneration output(rr);
        Layer_assignment layerAssignement(congestion, output);
        return output;
    }
};
} //end namespace NTHUR

#endif /* SRC_ROUTER_ROUTE_H_ */
