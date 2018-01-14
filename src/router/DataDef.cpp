/*
 * DataDef.cpp
 *
 *  Created on: Nov 29, 2017
 *      Author: florian
 */

/***********************
 * Global Variable Begin
 * ********************/
#include "DataDef.h"

namespace NTHUR {


Edge_2d::Edge_2d() :
        cost(0.0), MMVisitFlag(-1), cur_cap(0.), max_cap(0.), history(1), used_net(128) {

}



bool Two_pin_element::comp_2pin_net(Two_pin_element&a, Two_pin_element&b) {
    int a_bbox_size = abs(a.pin1.x - a.pin2.x) + abs(a.pin1.y - a.pin2.y);
    int b_bbox_size = abs(b.pin1.x - b.pin2.x) + abs(b.pin1.y - b.pin2.y);
    return (a_bbox_size < b_bbox_size);
}

bool Two_pin_element_2d::comp_2pin_net_from_path(Two_pin_element_2d&a, Two_pin_element_2d&b) {
    int a_bbox_size = abs(a.pin1.x - a.pin2.x) + abs(a.pin1.y - a.pin2.y);
    int b_bbox_size = abs(b.pin1.x - b.pin2.x) + abs(b.pin1.y - b.pin2.y);
    return (a_bbox_size < b_bbox_size);
}

} // namespace NTHUR
