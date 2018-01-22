#ifndef INC_ROUTINGREGION_H
#define INC_ROUTINGREGION_H

#include <cstddef>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../misc/geometry.h"
#include "EdgePlane3d.h"
#include "RoutingComponent.h"

namespace NTHUR {

class RoutingRegion {
public:

    RoutingRegion(int x, int y, int z);

//Get Functions
// Basic Information
    int get_gridx() const;        //get x grids (tile)
    int get_gridy() const;        //get y grids (tile)
    int get_layerNumber() const;        //get layer number
    int get_llx() const;        //get x-coordinate of lower left corner
    int get_lly() const;        //get y-coordinate of lower left corner
    int get_tileWidth() const;        //get tile width
    int get_tileHeight() const;        //get tile height
    std::size_t get_netNumber() const;        //get net number

    const Net& get_net(int netId) const;

// Pin list

    const EdgePlane3d<int>& getMaxCapacity() const;

public:

    void setVerticalCapacity(int layerId, int capacity);
    void setHorizontalCapacity(int layerId, int capacity);
    void setNetNumber(unsigned int netNumber);
    void adjustEdgeCapacity(int x1, int y1, int z1, int x2, int y2, int z2, int capacity);
    void setLayerMinimumWidth(unsigned int layerId, unsigned int width);
    void setLayerMinimumSpacing(unsigned int layerId, unsigned int spacing);
    void setViaSpacing(unsigned int layerId, unsigned int viaSpacing);
    void setTileTransformInformation(unsigned int llx, unsigned int lly, unsigned int tWidth, unsigned int tHeight);
    void beginAddANet(const std::string& netName, unsigned int netSerial, unsigned int pinNumber, unsigned int minWidth);
    void addPin(unsigned int x, unsigned int y, unsigned int layer);
    void endAddANet();

private:
    std::vector<Net> netList_;
    EdgePlane3d<int> max_capacity;
    int tileWidth;
    int tileHeight;
    int originX;
    int originY;

    int max_z;
    int max_x;
    int max_y;

    std::vector<int> wireWidth;        //minimum wire width
    std::vector<int> wireSpacing;        //minimum wire spacing
    std::vector<int> viaSpacing;        //minimum via spacing
//First int is the net id given from input file,
//the second id is the net position in NetList
    typedef std::unordered_map<int, int> NetIdLookupTable;
    NetIdLookupTable netSerial2NetId_;

    typedef std::unordered_set<Coordinate_2d> PinTable;
    PinTable pinTable_;
}
;


//Inline Functions
inline
int RoutingRegion::get_gridx() const {
    return max_x;
}

inline
int RoutingRegion::get_gridy() const {
    return max_y;
}

inline
int RoutingRegion::get_layerNumber() const {
    return max_z;
}

inline std::size_t RoutingRegion::get_netNumber() const {
    return netList_.size();
}

inline
int RoutingRegion::get_llx() const {
    return originX;
}

inline
int RoutingRegion::get_lly() const {
    return originY;
}

inline
int RoutingRegion::get_tileWidth() const {
    return tileWidth;
}

inline
int RoutingRegion::get_tileHeight() const {
    return tileHeight;
}

inline const Net& RoutingRegion::get_net(int netId) const {
    return netList_[netId];
}

inline
void RoutingRegion::setLayerMinimumWidth(unsigned int layerId, unsigned int width) {
    wireWidth[layerId] = width;
}

inline
void RoutingRegion::setLayerMinimumSpacing(unsigned int layerId, unsigned int spacing) {
    wireSpacing[layerId] = spacing;
}

inline
void RoutingRegion::setViaSpacing(unsigned int layerId, unsigned int value) {
    viaSpacing[layerId] = value;
}

inline const EdgePlane3d<int>& RoutingRegion::getMaxCapacity() const {
    return max_capacity;
}

} // namespace NTHUR

#endif /*INC_ROUTINGREGION_H*/
