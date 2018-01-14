#ifndef INC_FLUTE_4_NTHUROUTE_H
#define INC_FLUTE_4_NTHUROUTE_H

#include <array>
#include <vector>

#include "../grdb/RoutingComponent.h"
#include "flute-ds.h"           // flute data structure

class Flute {
public:
    Flute();

    void routeNet(const std::vector<Pin>& pinList, Tree& routingTree);

    void printTree(Tree& routingTree);
    void plotTree(Tree& routingTree);
    int treeWireLength(Tree& routingTree);

private:
    std::array<DTYPE, MAXD> x_;             ///< temporal integer array used by flute
    std::array<DTYPE, MAXD> y_;             ///< temporal integer array used by flute
};

#endif //INC_FLUTE_4_NTHUROUTE_H
