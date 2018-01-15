#ifndef INC_ROUTINGCOMPONENT_H
#define INC_ROUTINGCOMPONENT_H

#include <string>
#include <vector>

#include "EdgePlane3d.h"
#include "plane.h"

namespace NTHUR {

class Net {

public:

    typedef Coordinate_3d Pin;

    //Constructor
    Net(const std::string& name, int id = 0, int pos = 0, int minWidth = 0);

    void set_name(const std::string&);	//set net name
    void add_pin(const Pin& pin_ptr); //add pin pointer that point to pin
    const std::vector<Net::Pin>& get_pinList() const; //get the pin pointer list of this net
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
    std::vector<Net::Pin> pin_list;
};

} // namespace NTHUR

#endif /*INC_ROUTINGCOMPONENT_H*/
