#ifndef INC_FLUTE_4_NTHUROUTE_H
#define INC_FLUTE_4_NTHUROUTE_H

#include "flute-ds.h"           // flute data structure
#include "flute-function.h"
#include "../grdb/RoutingComponent.h"
#include "../misc/geometry.h"

class Flute {
    public:
                Flute ();
                ~Flute ();

    void        routeNet (const std::vector<Pin>& pinList, Tree& routingTree);

    void        printTree (Tree& routingTree);

    int         treeWireLength (Tree& routingTree);

    private:
    DTYPE*      x_;             ///< temporal integer array used by flute
    DTYPE*      y_;             ///< temporal integer array used by flute
};

#endif //INC_FLUTE_4_NTHUROUTE_H
