#ifndef INC_ROUTINGCOMPONENT_H
#define INC_ROUTINGCOMPONENT_H

#include <string>
#include <vector>

#include "plane.h"

namespace NTHUR {


class Pin {
public:
    //Constructor
    Pin(int x = 0, int y = 0, int z = 0);
    //  Pin();
    int get_tileX() const;
    int get_tileY() const;
    int get_layerId() const;		//get the layer id that contain this pin
    Coordinate_2d get_tileXY() const;
    void x(int i);
    void y(int i);
    void z(int i);
    int x() const;
    int y() const;
    int z() const;

    std::string toString() const;

protected:
    int x_;
    int y_;
    int z_;
};

inline
void Pin::x(int i) {
    x_ = i;
}

inline
void Pin::y(int i) {
    y_ = i;
}

inline
void Pin::z(int i) {
    z_ = i;
}

inline
int Pin::x() const {
    return x_;
}

inline
int Pin::y() const {
    return y_;
}

inline
int Pin::z() const {
    return z_;
}

inline std::string Pin::toString() const {
    return std::to_string(x_) + "," + std::to_string(y_) + "," + std::to_string(z_);
}

class Net {

public:
    //Constructor
    Net(const std::string& name, int id = 0, int pos = 0, int minWidth = 0);

    void set_name(const std::string&);	//set net name
    void add_pin(const Pin& pin_ptr); //add pin pointer that point to pin
    const std::vector<Pin>& get_pinList() const; //get the pin pointer list of this net
    const std::string& get_name() const;	//get net name
    int get_pinNumber() const;		//get pin number in this net
    int get_bboxSize() const;      //get the bounding box size

    static bool comp_net(const Net& a, const Net& b);

    std::string toString() const;

public:
    int serialNumber;   //net id_# in test case
    int id;             //the net ordered in test case
    int minWireWidth;	//minimum wire width of this net

private:
    std::string name;
    int minPinX;
    int maxPinX;
    int minPinY;
    int maxPinY;
    std::vector<Pin> pin_list;
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
    std::vector<Plane<Pin, int> > routingSpace_;
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
    return routingSpace_[z];
}

inline
int RoutingSpace::getXSize() const {
    return routingSpace_[0].getXSize();
}

inline
int RoutingSpace::getYSize() const {
    return routingSpace_[0].getYSize();
}

inline
int RoutingSpace::getZSize() const {
    return routingSpace_.size();
}

} // namespace NTHUR

#endif /*INC_ROUTINGCOMPONENT_H*/
