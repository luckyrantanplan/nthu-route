#include<iostream>
#include "RoutingRegion.h"
#include "../misc/geometry.h"

namespace NTHUR {

/***************
 RoutingRegion
 **************/

void RoutingRegion::setVerticalCapacity(unsigned int layerId, unsigned int capacity) {
    for (int x = 0; x < routingSpace_.getXSize(); ++x) {
        for (int y = 0; y < routingSpace_.getYSize(); ++y) {
            routingSpace_.south(Coordinate_3d { x, y, layerId }) = capacity;
        }
    }
}

void RoutingRegion::setHorizontalCapacity(unsigned int layerId, unsigned int capacity) {
    for (int x = 0; x < routingSpace_.getXSize(); ++x) {
        for (int y = 0; y < routingSpace_.getYSize(); ++y) {
            routingSpace_.east(Coordinate_3d { x, y, layerId }) = capacity;
        }
    }
}

void RoutingRegion::setNetNumber(unsigned int netNumber) {
    netList_.clear();
    netList_.reserve(netNumber);
    netSerial2NetId_.reserve(netNumber);

}

void RoutingRegion::adjustEdgeCapacity(unsigned int x1, unsigned int y1, unsigned int z1, unsigned int x2, unsigned int y2, unsigned int z2, unsigned int capacity) {

    routingSpace_.edge(Coordinate_3d(x1, y1, z1), Coordinate_3d(x2, y2, z2)) = capacity;

}

void RoutingRegion::setTileTransformInformation(unsigned int llx, unsigned int lly, unsigned int tWidth, unsigned int tHeight) {
    originX = llx;
    originY = lly;
    tileWidth = tWidth;
    tileHeight = tHeight;
}

void RoutingRegion::beginAddANet(const char* netName, unsigned int netSerial, unsigned int, //pinNumber,
        unsigned int minWidth) {
    int netId = netList_.size();
    netSerial2NetId_[netSerial] = netId;
    netList_.push_back(Net(netName, netSerial, netId, minWidth));
}

void RoutingRegion::addPin(unsigned int x, unsigned int y, unsigned int layer) {

//transfer pin's coordinate to tile position
    int tileX = ((x - get_llx()) / get_tileWidth());
    int tileY = ((y - get_lly()) / get_tileHeight());

    if (pinTable_.find(std::pair<int, int>(tileX, tileY)) == pinTable_.end()) {
        pinTable_.insert(std::pair<int, int>(tileX, tileY));

        netList_.back().add_pin(Pin(tileX, tileY, layer));
    }
}

void RoutingRegion::endAddANet() {
    if (netList_.back().get_pinList().size() <= 1) {
        netList_.pop_back();
    }
    pinTable_.clear();
}

} // namespace NTHUR
