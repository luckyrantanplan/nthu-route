#include "traversemap.h"
#include "../grdb/RoutingRegion.h"
#include "../misc/geometry.h"

#include <iostream>
#include <vector>

using namespace std;

/****************
 * BSearchQueNode
 ****************/
BSearchQueNode::BSearchQueNode(int x, int y, int z, int wirePos) :
        cell(x, y, z), wirePos(wirePos) {
}
