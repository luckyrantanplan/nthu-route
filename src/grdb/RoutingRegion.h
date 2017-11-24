#ifndef INC_ROUTINGREGION_H
#define INC_ROUTINGREGION_H

#include "RoutingComponent.h"
#include "builder.h"

#include <list>
#include <vector>
#include <set>
#include <unordered_map>

class RoutingRegion : public Builder {
	public:
    //Constructor
    RoutingRegion();
    ~RoutingRegion();
    
    //Get Functions
    // Basic Information
    int get_gridx();				//get x grids (tile)
    int get_gridy();				//get y grids (tile)
    int get_layerNumber();			//get layer number
    int get_layerMinWidth(int layer_id);	//get minimum wire width of the specified layer
    int get_layerMinSpacing(int layer_id);	//get minimum spacing of the specified layer
    int get_layerViaSpacing(int layer_id);	//get via spacing of the specified layer
    int get_llx();							//get x-coordinate of lower left corner
    int get_lly();					//get y-coordinate of lower left corner
    int get_tileWidth();			//get tile width
    int get_tileHeight();			//get tile height
    int get_netNumber();			//get net number
    const char* get_netName(int netPos);	//get net name
    int get_netSerialNumber(int netId);
    int get_netId(int netSerial);
    //int get_netPos(int net_id);	//get net's list position by net id
    int get_netPinNumber(int netPos);		//get pin number of the specified net
    int get_netMinWidth(int netPos);		//get minimum wire width of the specified net
    NetList& get_netList();

    // Pin list
    const PinptrList& get_nPin(int net_id);	//get Pins by net

    // Edges' capacity information
    int capacity(int layer_id, int x1, int y1, int x2, int y2);
    //int curCapacity(int layer_id, int x1, int y1, int x2, int y2);
    //void inc_curCapacity(int layer_id, int x1, int y1, int x2, int y2);


    public:
    virtual void    setGrid (unsigned int x,
                             unsigned int y,
                             unsigned int layerNumber);
    virtual void    setVerticalCapacity (unsigned int layerId,
                                         unsigned int capacity);
    virtual void    setHorizontalCapacity (unsigned int layerId,
                                           unsigned int capacity);
    virtual void    setNetNumber (unsigned int netNumber);
    virtual void    adjustEdgeCapacity (unsigned int x1,
                                        unsigned int y1,
                                        unsigned int z1,
                                        unsigned int x2,
                                        unsigned int y2,
                                        unsigned int z2,
                                        unsigned int capacity);
    virtual void    setLayerMinimumWidth (unsigned int layerId,
                                          unsigned int width);
    virtual void    setLayerMinimumSpacing (unsigned int layerId,
                                            unsigned int spacing);
    virtual void    setViaSpacing (unsigned int layerId,
                                   unsigned int viaSpacing);
    virtual void    setTileTransformInformation (unsigned int llx,
                                                 unsigned int lly, 
                                                 unsigned int tWidth, 
                                                 unsigned int tHeight);
    virtual void 	beginAddANet (const char* netName,
                                  unsigned int netSerial,
                                  unsigned int pinNumber,
                                  unsigned int minWidth);
    virtual void    addPin (unsigned int x,
                            unsigned int y,
                            unsigned int layer);
    virtual void    endAddANet ();
    virtual void    endBuild ();

	private:
    NetList* netList_;
    RoutingSpace* routingSpace_;

    //First int is the net id given from input file,
    //the second id is the net position in NetList
    typedef std::unordered_map<int, int> NetIdLookupTable;
    NetIdLookupTable* netSerial2NetId_;

    typedef std::set< std::pair<int, int> > PinTable;
    PinTable*   pinTable_;
};

//Inline Functions
inline
int RoutingRegion::get_gridx()
{
    return routingSpace_->getXSize();
}

inline
int RoutingRegion::get_gridy()
{
    return routingSpace_->getYSize();
}

inline
int RoutingRegion::get_layerNumber()
{
    return routingSpace_->getZSize();
}

inline
int RoutingRegion::get_layerMinWidth(int layer_id){
	return routingSpace_->wireWidth[layer_id];
}

inline
int RoutingRegion::get_layerMinSpacing(int layer_id){
	return routingSpace_->wireSpacing[layer_id];
}

inline
int RoutingRegion::get_layerViaSpacing(int layer_id){
	return routingSpace_->viaSpacing[layer_id];
}

inline
int RoutingRegion::get_llx(){
	return routingSpace_->originX;
}

inline
int RoutingRegion::get_lly(){
	return routingSpace_->originY;
}

inline
int RoutingRegion::get_tileWidth(){
	return routingSpace_->tileWidth;
}

inline
int RoutingRegion::get_tileHeight(){
	return routingSpace_->tileHeight;
}

inline
int RoutingRegion::get_netNumber(){
	return netList_->size();
}

inline
const char* RoutingRegion::get_netName(int netId){
	return (*netList_)[netId].get_name();
}

inline
int RoutingRegion::get_netSerialNumber(int netId){
	return (*netList_)[netId].id;
}

inline
int RoutingRegion::get_netId(int net_id){
	return (*netSerial2NetId_)[net_id];
}

inline
int RoutingRegion::get_netPinNumber(int netId){
	return (*netList_)[netId].get_pinNumber();
}

inline
int RoutingRegion::get_netMinWidth(int netId){
	return (*netList_)[netId].minWireWidth;
}

inline
NetList& RoutingRegion::get_netList ()
{
    return (*netList_);
}


inline
const PinptrList& RoutingRegion::get_nPin(int netId){	//get Pins by net
	return (*netList_)[netId].get_pinList();
}

inline
void RoutingRegion::setLayerMinimumWidth (unsigned int layerId,
                                          unsigned int width)
{
    routingSpace_->wireWidth[layerId] = width;
}

inline
void RoutingRegion::setLayerMinimumSpacing (unsigned int layerId,
                                            unsigned int spacing)
{
    routingSpace_->wireSpacing[layerId] = spacing;
}

inline
void RoutingRegion::setViaSpacing (unsigned int layerId,
                                   unsigned int viaSpacing)
{
    routingSpace_->viaSpacing[layerId] = viaSpacing;
}

#endif /*INC_ROUTINGREGION_H*/
