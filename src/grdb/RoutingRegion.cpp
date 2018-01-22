#include<iostream>
#include "RoutingRegion.h"
#include "../misc/geometry.h"

namespace NTHUR {

/***************
 RoutingRegion
 **************/
RoutingRegion::RoutingRegion(int x, int y, int z) :
        max_capacity { Coordinate_3d { x, y, z } }, //
        tileWidth(1), //
        tileHeight(1), //
        originX(0), //
        originY(0), //
        wireWidth { z }, //
        wireSpacing { z }, //
        viaSpacing { z } //
{
    max_z = z;
    max_x = x;
    max_y = y;
    for (int x = 0; x < max_x; ++x) {
        for (int y = 0; y < max_y; ++y) {
            for (int z = 0; z < max_z; ++z) {
                max_capacity.front(Coordinate_3d { x, y, z }) = std::numeric_limits<int>::max();
            }
        }
    }
}

void RoutingRegion::setVerticalCapacity(int layerId, int capacity) {
    for (int x = 0; x < max_capacity.getXSize(); ++x) {
        for (int y = 0; y < max_capacity.getYSize(); ++y) {
            max_capacity.south(Coordinate_3d { x, y, layerId }) = capacity;
        }
    }
}

void RoutingRegion::setHorizontalCapacity(int layerId, int capacity) {
    for (int x = 0; x < max_capacity.getXSize(); ++x) {
        for (int y = 0; y < max_capacity.getYSize(); ++y) {
            max_capacity.east(Coordinate_3d { x, y, layerId }) = capacity;
        }
    }
}

void RoutingRegion::setNetNumber(unsigned int netNumber) {
    netList_.clear();
    netList_.reserve(netNumber);
    netSerial2NetId_.reserve(netNumber);

}

void RoutingRegion::adjustEdgeCapacity(int x1, int y1, int z1, int x2, int y2, int z2, int capacity) {

    max_capacity.edge(Coordinate_3d(x1, y1, z1), Coordinate_3d(x2, y2, z2)) = capacity;

}

void RoutingRegion::setTileTransformInformation(unsigned int llx, unsigned int lly, unsigned int tWidth, unsigned int tHeight) {
    originX = llx;
    originY = lly;
    tileWidth = tWidth;
    tileHeight = tHeight;
}

void RoutingRegion::beginAddANet(const std::string& netName, unsigned int netSerial, unsigned int, //pinNumber,
        unsigned int minWidth) {
    int netId = netList_.size();
    netSerial2NetId_[netSerial] = netId;
    netList_.push_back(Net(netName, netSerial, netId, minWidth));
}

void RoutingRegion::addPin(unsigned int x, unsigned int y, unsigned int layer) {

//transfer pin's coordinate to tile position
    int tileX = ((x - get_llx()) / get_tileWidth());
    int tileY = ((y - get_lly()) / get_tileHeight());

    std::pair<PinTable::const_iterator, bool> pair = pinTable_.emplace(tileX, tileY);
    if (pair.second) {
        netList_.back().add_pin(Net::Pin(tileX, tileY, layer));
    }

}

void RoutingRegion::endAddANet() {
    if (netList_.back().get_pinList().size() <= 1) {
        netList_.pop_back();
    }
    pinTable_.clear();
}

} // namespace NTHUR
