#include "flute4nthuroute.h"
#include "../misc/debug.h"

Flute::Flute() :
        x_(NULL), y_(NULL) {
    readLUT();      //Read in the binary lookup table - POWVFILE, POSTFILE

    x_ = new DTYPE[MAXD];    //int array used as input of FLUTE
    y_ = new DTYPE[MAXD];    //int array used as input of FLUTE
}

Flute::~Flute() {
    if (x_ != NULL) {
        delete[] x_;
    }
    if (y_ != NULL) {
        delete[] y_;
    }
}

void Flute::routeNet(const std::vector<Pin>& pinList, Tree& routingTree) {
    int pinNumber = pinList.size();

    //The pin number must <= MAXD, or the flute will crash
    assert(pinNumber <= MAXD);

    // insert 2D-coordinate of pins of a net into x_ and y_
    for (int pinId = 0; pinId < pinNumber; ++pinId) {
        x_[pinId] = pinList[pinId].x();
        y_[pinId] = pinList[pinId].y();
    }

    // obtain the routing tree by FLUTE
    routingTree = flute(pinNumber, x_, y_, ACCURACY);
}

void Flute::printTree(Tree& routingTree) {
    printtree(routingTree);
}

int Flute::treeWireLength(Tree& routingTree) {
    return static_cast<int>(wirelength(routingTree));
}
