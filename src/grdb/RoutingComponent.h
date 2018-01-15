#ifndef INC_ROUTINGCOMPONENT_H
#define INC_ROUTINGCOMPONENT_H

#include <string>
#include <vector>

#include "EdgePlane3d.h"
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

} // namespace NTHUR

#endif /*INC_ROUTINGCOMPONENT_H*/
