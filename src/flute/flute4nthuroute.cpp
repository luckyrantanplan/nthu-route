#include "flute4nthuroute.h"

#include <cassert>

#include "flute-function.h"

Flute::Flute() {
    readLUT();      //Read in the binary lookup table - POWVFILE, POSTFILE

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
    routingTree = flute(pinNumber, x_.data(), y_.data(), ACCURACY);
}

void Flute::printTree(Tree& routingTree) {
    printtree(routingTree);
}

void Flute::plotTree(Tree& routingTree) {
    plottree(routingTree);
}

int Flute::treeWireLength(Tree& routingTree) {
    return static_cast<int>(wirelength(routingTree));
}
