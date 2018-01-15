#include "RoutingComponent.h"
#include "../misc/geometry.h"

#include <climits>

namespace NTHUR {

/******
 Net
 *****/
Net::Net(const std::string& name, int serialNumber, int id, int minWireWidth) :
        serialNumber(serialNumber), id(id), minWireWidth(minWireWidth), name(name), minPinX(INT_MAX), maxPinX(0), minPinY(INT_MAX), maxPinY(0) {
}

void Net::set_name(const std::string& iname) {
    name = iname;
}

std::string Net::toString() const {
    std::string s = "serialNumber: " + std::to_string(serialNumber);
    s += " id: " + std::to_string(id);
    s += " minWireWidth: " + std::to_string(minWireWidth);
    s += " name: " + name;
    s += " minPinX: " + std::to_string(minPinX);
    s += " maxPinX: " + std::to_string(maxPinX);
    s += " minPinY: " + std::to_string(minPinY);
    s += " maxPinY: " + std::to_string(maxPinY);
    s += " pin_list: [";
    for (const Pin& p : pin_list) {
        s += "(" + p.toString() + ") ";
    }
    s += "]";
    return s;
}

void Net::add_pin(const Pin& pin) {
    //Update boundy box information
    if (pin.x < minPinX) {
        minPinX = pin.x;
    }
    if (pin.x > maxPinX) {
        maxPinX = pin.x;
    }

    if (pin.y < minPinY) {
        minPinY = pin.y;
    }
    if (pin.y > maxPinY) {
        maxPinY = pin.y;
    }

    //push the new pin to pin list
    pin_list.push_back(pin);
}

const std::vector<Net::Pin>& Net::get_pinList() const {
    return pin_list;
}

const std::string& Net::get_name() const {
    return name;
}

int Net::get_bboxSize() const {
    if (!pin_list.empty()) {
        return (maxPinX - minPinX) + (maxPinY - minPinY);
    }

    return 0;
}

/*sort bbox in ascending order, then pin_num in descending order*/
bool Net::comp_net(const Net& a, const Net& b) {
    if (a.get_bboxSize() > b.get_bboxSize()) {
        return true;
    } else if (a.get_bboxSize() < b.get_bboxSize()) {
        return false;
    } else {
        return (a.pin_list.size() < b.pin_list.size());
    }
}

} // namespace NTHUR
