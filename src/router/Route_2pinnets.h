#ifndef INC_REOUTE_2PINNETS_H
#define INC_REOUTE_2PINNETS_H

#include "Construct_2d_tree.h"
#include "../grdb/plane.h"

class Point_fc {
public:
    Point_fc(int x = 0, int y = 0) :
            x(x), y(y) {
    }

public:
    int x;
    int y;
    vector<Two_pin_element_2d*> points;
};

extern VertexPlane<Point_fc>* gridcell;

extern void route_all_2pin_net(void);
extern void allocate_gridcell();
extern void reallocate_two_pin_list(bool insert_to_branch);

#endif //INC_REOUTE_2PINNETS_H
