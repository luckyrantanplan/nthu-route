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
