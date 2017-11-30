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

Edge_2d::Edge_2d() :
        cur_cap(0.), max_cap(0.), history(1), used_net(128) {

}

Edge_3d::Edge_3d() :
        max_cap(0), cur_cap(0), used_net(5) {

}

bool Vertex_flute::comp_vertex_fl(const Vertex_flute& a, const Vertex_flute& b) const {
    if (a.x < b.x)
        return true;
    else if (a.x > b.x)
        return false;
    else if (a.y < b.y)
        return true;
    else if (a.y > b.y)
        return false;
    else if (a.type == PIN)
        return true;
    else
        return false;
}
