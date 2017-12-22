#ifndef INC_ROUTINGREGION_H
#define INC_ROUTINGREGION_H

#include "RoutingComponent.h"

#include <list>
#include <vector>
#include <set>
#include <unordered_map>

class RoutingRegion {
public:

    //Get Functions
    // Basic Information
    int get_gridx() const;				//get x grids (tile)
    int get_gridy() const;				//get y grids (tile)
    int get_layerNumber() const;			//get layer number
    int get_llx() const;							//get x-coordinate of lower left corner
    int get_lly() const;					//get y-coordinate of lower left corner
    int get_tileWidth() const;			//get tile width
    int get_tileHeight() const;			//get tile height
    int get_netNumber() const;                 //get net number

    const std::string& get_netName(int netPos) const;	//get net name

    int get_netPinNumber(int netPos) const;		//get pin number of the specified net

    NetList& get_netList();

    // Pin list
    const std::vector<Pin>& get_nPin(int net_id) const;	//get Pins by net
    std::string nPinToString(int net_id) const;

    Plane<Pin, int>& getLayer(int z);
    const Plane<Pin, int>& getLayer(int z) const;

public:
    void setGrid(unsigned int x, unsigned int y, unsigned int layerNumber);
    void setVerticalCapacity(unsigned int layerId, unsigned int capacity);
    void setHorizontalCapacity(unsigned int layerId, unsigned int capacity);
    void setNetNumber(unsigned int netNumber);
    void adjustEdgeCapacity(unsigned int x1, unsigned int y1, unsigned int z1, unsigned int x2, unsigned int y2, unsigned int z2, unsigned int capacity);
    void setLayerMinimumWidth(unsigned int layerId, unsigned int width);
    void setLayerMinimumSpacing(unsigned int layerId, unsigned int spacing);
    void setViaSpacing(unsigned int layerId, unsigned int viaSpacing);
    void setTileTransformInformation(unsigned int llx, unsigned int lly, unsigned int tWidth, unsigned int tHeight);
    void beginAddANet(const char* netName, unsigned int netSerial, unsigned int pinNumber, unsigned int minWidth);
    void addPin(unsigned int x, unsigned int y, unsigned int layer);
    void endAddANet();
    void endBuild();

private:
    NetList netList_;
    RoutingSpace routingSpace_;

    //First int is the net id given from input file,
    //the second id is the net position in NetList
    typedef std::unordered_map<int, int> NetIdLookupTable;
    NetIdLookupTable netSerial2NetId_;

    typedef std::set<std::pair<int, int> > PinTable;
    PinTable pinTable_;
};

//Inline Functions
inline
int RoutingRegion::get_gridx() const {
    return routingSpace_.getXSize();
}

inline
int RoutingRegion::get_gridy() const {
    return routingSpace_.getYSize();
}

inline
int RoutingRegion::get_layerNumber() const {
    return routingSpace_.getZSize();
}
inline
int RoutingRegion::get_netNumber() const {
    return netList_.size();
}

inline
int RoutingRegion::get_llx() const {
    return routingSpace_.originX;
}

inline
int RoutingRegion::get_lly() const {
    return routingSpace_.originY;
}

inline
int RoutingRegion::get_tileWidth() const {
    return routingSpace_.tileWidth;
}

inline
int RoutingRegion::get_tileHeight() const {
    return routingSpace_.tileHeight;
}

inline const std::string& RoutingRegion::get_netName(int netId) const {
    return netList_[netId].get_name();
}

inline
int RoutingRegion::get_netPinNumber(int netId) const {
    return netList_[netId].get_pinNumber();
}

inline NetList& RoutingRegion::get_netList() {
    return netList_;
}

inline const std::vector<Pin>& RoutingRegion::get_nPin(int netId) const {	//get Pins by net
    return netList_[netId].get_pinList();
}

inline
void RoutingRegion::setLayerMinimumWidth(unsigned int layerId, unsigned int width) {
    routingSpace_.wireWidth[layerId] = width;
}

inline
void RoutingRegion::setLayerMinimumSpacing(unsigned int layerId, unsigned int spacing) {
    routingSpace_.wireSpacing[layerId] = spacing;
}

inline
void RoutingRegion::setViaSpacing(unsigned int layerId, unsigned int viaSpacing) {
    routingSpace_.viaSpacing[layerId] = viaSpacing;
}

inline std::string RoutingRegion::nPinToString(int net_id) const {
    return netList_[net_id].toString();
}

inline const Plane<Pin, int>& RoutingRegion::getLayer(int z) const {
    return routingSpace_.layer(z);
}

inline Plane<Pin, int>& RoutingRegion::getLayer(int z) {
    return routingSpace_.layer(z);

}
#endif /*INC_ROUTINGREGION_H*/
