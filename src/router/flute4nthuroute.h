#ifndef INC_FLUTE_4_NTHUROUTE_H
#define INC_FLUTE_4_NTHUROUTE_H

#include <array>
#include <vector>

#include "../grdb/RoutingComponent.h"
#include "../flute/flute-ds.h"           // flute data structure

namespace NTHUR {

struct TreeFlute {
    int deg;          // degree
    DTYPE length;     // total wirelength
    std::vector<Branch> branch;   // array of tree branches

    int number;

    void set(const Tree& tree) {
        deg = tree.deg;
        length = tree.length;
        number = tree.number;
        branch.resize(2 * deg - 2);
        for (int i = 0; i < 2 * deg - 2; ++i) {
            branch[i] = tree.branch[i];
        }
    }

    std::string plot() const {
        std::string s;
        for (int i = 0; i < number; ++i) {
            const Branch& b = branch[i];
            s += std::to_string(static_cast<int>(b.x)) + " " + std::to_string(static_cast<int>(b.y)) + "\n";
            s += std::to_string(static_cast<int>(branch[b.n].x)) + " " + std::to_string(static_cast<int>(branch[b.n].y)) + "\n\n";
        }

        return s;
    }
};

struct TreeWrapper {

    Tree tree;

    TreeWrapper() {
        tree.branch = nullptr;
    }
    ~TreeWrapper() {
        if (tree.branch != nullptr) {
            free(tree.branch);
        }
    }
};

class Flute {
public:
    Flute();

    void routeNet(const std::vector<Net::Pin>& pinList, TreeFlute& result);

    void printTree(Tree& routingTree);
    void plotTree(Tree& routingTree);
    int treeWireLength(Tree& routingTree);

private:
    std::array<DTYPE, MAXD> x_;             ///< temporal integer array used by flute
    std::array<DTYPE, MAXD> y_;             ///< temporal integer array used by flute
};
} // namespace NTHUR
#endif //INC_FLUTE_4_NTHUROUTE_H
