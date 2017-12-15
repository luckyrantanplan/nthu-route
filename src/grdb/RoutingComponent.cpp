#include "RoutingComponent.h"
#include "../misc/geometry.h"
#include "../misc/debug.h"

#include <climits>

using namespace std;

/*******
 Pin
 ******/
Pin::Pin(int x, int y, int z) :
        Coordinate(x, y, z) {
}
//Pin::Pin() :
//        Coordinate { 0, 0, 0 } {
//}
int Pin::get_tileX() const {
    return x();
}

int Pin::get_tileY() const {
    return y();
}

int Pin::get_layerId() const {
    return z();
}

Coordinate_2d Pin::get_tileXY() const {
    return Coordinate_2d { x(), y() };
}
/******
 Net
 *****/
Net::Net(const char* name, int serialNumber, int id, int minWireWidth) :
        serialNumber(serialNumber), id(id), minWireWidth(minWireWidth), name(name), minPinX(INT_MAX), maxPinX(0), minPinY(INT_MAX), maxPinY(0) {
}

void Net::set_name(const char* name) {
    this->name = name;
}

void Net::add_pin(const Pin* pin) {
    //Update boundy box information
    if (pin->x() < minPinX) {
        minPinX = pin->x();
    }
    if (pin->x() > maxPinX) {
        maxPinX = pin->x();
    }

    if (pin->y() < minPinY) {
        minPinY = pin->y();
    }
    if (pin->y() > maxPinY) {
        maxPinY = pin->y();
    }

    //push the new pin to pin list
    this->pin_list.push_back(pin);
}

const PinptrList& Net::get_pinList() const {
    return this->pin_list;
}

const std::string& Net::get_name() const {
    return this->name ;
}

int Net::get_pinNumber() const {
    return this->pin_list.size();
}

int Net::get_bboxSize() const {
    if (get_pinNumber() > 0) {
        return (maxPinX - minPinX) + (maxPinY - minPinY);
    }

    return 0;
}

int Net::get_boundaryN() const {
    return maxPinY;
}

int Net::get_boundaryE() const {
    return maxPinX;
}

int Net::get_boundaryW() const {
    return minPinX;
}

int Net::get_boundaryS() const {
    return minPinY;
}

/*sort bbox in ascending order, then pin_num in descending order*/
bool Net::comp_net(const Net& a, const Net& b) {
    if (a.get_bboxSize() > b.get_bboxSize()) {
        return true;
    } else if (a.get_bboxSize() < b.get_bboxSize()) {
        return false;
    } else {
        return (a.get_pinNumber() < b.get_pinNumber());
    }
}

/*************
 * RoutingSpace
 *************/
RoutingSpace::RoutingSpace(int x, int y, int z) :
        tileWidth(1), tileHeight(1), //
        originX(0), originY(0), wireWidth(z), wireSpacing(z), viaSpacing(z), //
        routingSpace_ { std::vector<Plane<Pin, int> >(z, Plane<Pin, int>(x, y)) }

{

    assignTileCoordinate();
}

RoutingSpace::~RoutingSpace() {

}

void RoutingSpace::assignTileCoordinate() {
    if (routingSpace_.size() > 0) {
        for (int z = 0; z < getZSize(); ++z) {
            for (int x = 0; x < getXSize(); ++x) {
                for (int y = 0; y < getYSize(); ++y) {
                    routingSpace_[z].vertex(x, y).x(x);
                    routingSpace_[z].vertex(x, y).y(y);
                    routingSpace_[z].vertex(x, y).z(z);
                }
            }
        }
    }
}
