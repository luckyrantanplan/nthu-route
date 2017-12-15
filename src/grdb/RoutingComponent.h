#ifndef INC_ROUTINGCOMPONENT_H
#define INC_ROUTINGCOMPONENT_H

#include "plane.h"
#include "../misc/geometry.h"

#include <utility>
#include <string>
#include <vector>

class Pin: public Coordinate {
public:
    //Constructor
    Pin(int x = 0, int y = 0, int z = 0);
    //  Pin();
    int get_tileX() const;
    int get_tileY() const;
    int get_layerId() const;		//get the layer id that contain this pin
    Coordinate_2d get_tileXY() const;
};

typedef std::vector<const Pin*> PinptrList;

class Net {
public:
    int serialNumber;   //net id_# in test case
    int id;             //the net ordered in test case
    int minWireWidth;	//minimum wire width of this net

public:
    //Constructor
    Net(const char* name = "", int id = 0, int pos = 0, int minWidth = 0);

    void set_name(const char*);	//set net name
    void add_pin(const Pin* pin_ptr); //add pin pointer that point to pin
    const PinptrList& get_pinList(); //get the pin pointer list of this net
    const std::string& get_name() const;	//get net name
    int get_pinNumber() const;		//get pin number in this net
    int get_bboxSize() const;      //get the bounding box size
    int get_boundaryN() const;
    int get_boundaryE() const;
    int get_boundaryW() const;
    int get_boundaryS() const;
    static bool comp_net(const Net& a, const Net& b);
private:
    std::string name;
    int minPinX;
    int maxPinX;
    int minPinY;
    int maxPinY;
    PinptrList pin_list;
};

class RoutingSpace {

public:
    int tileWidth;
    int tileHeight;
    int originX;
    int originY;
    std::vector<int> wireWidth;		//minimum wire width
    std::vector<int> wireSpacing;	//minimum wire spacing
    std::vector<int> viaSpacing;	//minimum via spacing

public:
    RoutingSpace(int x = 0, int y = 0, int z = 0);
    ~RoutingSpace();

    void resize(int x, int y, int z);

    ///Get the size of routing spacing in x-axis
    int getXSize() const;

    ///Get the size of routing spacing in y-axis
    int getYSize() const;

    ///Get the size of routing spacing in z-axis
    int getZSize() const;

    ///@brief Get the specified tile
    Pin& tile(int x, int y, int z);

    ///@brief Get the specified tile, and the tile is read-only.
    const Pin& tile(int x, int y, int z) const;

    ///@brief Get the specified edge
    Plane<Pin, int>& layer(int z);

    ///@brief Get the specified edge, and the edge is read-only.
    const Plane<Pin, int>& layer(int z) const;

private:

private:
    std::vector<Plane<Pin, int> > routingSpace_;

private:
    void assignTileCoordinate();
};

typedef std::vector<Net> NetList;

//Inline Functions
/*************
 * RoutingSpace
 *************/
inline
void RoutingSpace::resize(int x, int y, int z) {
    //routingSpace_.resize(x, y , z);
    routingSpace_.resize(z, Plane<Pin, int>(x, y));
    wireWidth.resize(z);
    wireSpacing.resize(z);
    viaSpacing.resize(z);
    assignTileCoordinate();
}

inline Pin& RoutingSpace::tile(int x, int y, int z) {
    return routingSpace_[z].vertex(x, y);
}

inline const Pin& RoutingSpace::tile(int x, int y, int z) const {
    return routingSpace_[z].vertex(x, y);
}

inline Plane<Pin, int>& RoutingSpace::layer(int z) {
    return routingSpace_[z];
}

inline const Plane<Pin, int> & RoutingSpace::layer(int z) const {
    return layer(z);
}

inline
int RoutingSpace::getXSize() const {
    assert(routingSpace_.size() > 0);
    return routingSpace_[0].getXSize();
}

inline
int RoutingSpace::getYSize() const {
    assert(routingSpace_.size() > 0);
    return routingSpace_[0].getYSize();
}

inline
int RoutingSpace::getZSize() const {
    //return routingSpace_.getZSize();
    return routingSpace_.size();
}

#endif /*INC_ROUTINGCOMPONENT_H*/
