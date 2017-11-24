//  File: grdb/builder.h
//  Brief: Provide an interface for parser to insert data into database
//  Author: Yen-Jung Chang
//  $Date: 2007-11-26 17:18:13 +0800 (Mon, 26 Nov 2007) $
//  $Revision: 13 $

#ifndef INC_BUILDER_H
#define INC_BUILDER_H

/**
 * @brief This class is an interface, which provide callback functions
 * for parser to insert data into your data structure.
*/
class Builder {
    public:
    virtual         ~Builder();

    /// @brief Set width, hight and layer number of routing layers
    /// @param[in] x Max tile number in x-axis
    /// @param[in] y Max tile number in y-axis
    /// @param[in] layerNumber Total layer number
    virtual void    setGrid (unsigned int x,
                             unsigned int y,
                             unsigned int layerNumber) = 0;

    /// Set capacity of vertical edges of specified layer
    virtual void    setVerticalCapacity (unsigned int layerId,
                                         unsigned int capacity) = 0;

    /// Set capacity of horizontal edges of specified layer
    virtual void    setHorizontalCapacity (unsigned int layerId,
                                           unsigned int capacity) = 0;

    /// Set the total net number in test case
    virtual void    setNetNumber (unsigned int netNumber) = 0;

    /// @brief Set capacity of sepecified edge
    /// @note The value of z starts from 0, so if your first layer
    /// is 1, don't forget minor 1 before passing it to this
    /// function
    virtual void    adjustEdgeCapacity (unsigned int x1,
                                        unsigned int y1,
                                        unsigned int z1,
                                        unsigned int x2,
                                        unsigned int y2,
                                        unsigned int z2,
                                        unsigned int capacity) = 0;

    /// Set minimun net width of the specified layer
    virtual void    setLayerMinimumWidth (unsigned int layerId,
                                          unsigned int width) = 0;

    /// Set minimun wire spacing of the specified layer
    virtual void    setLayerMinimumSpacing (unsigned int layerId,
                                            unsigned int spacing) = 0;

    /// Set via spacing of specified layer
    virtual void    setViaSpacing (unsigned int layerId,
                                   unsigned int viaSpacing) = 0;

    ///@brief Set the transfor information of the routing layers
    ///@details In ISPD'07 Global Routing Contest, the coordinate
    /// of a pin is not the coordinate on global routing layer.
    /// So we need do some tranfrom, i.e., realX = (x - llx)/tWidth
    ///@param[in] llx Lower left x, the origin's x-axis
    ///@param[in] lly Lower left y, the origin's y-axis
    ///@param[in] tWidth Tile width
    ///@param[in] tHeight Tile height
    virtual void    setTileTransformInformation (unsigned int llx,
                                                 unsigned int lly, 
                                                 unsigned int tWidth, 
                                                 unsigned int tHeight) = 0;

    ///@brief Begin to add a net (we call it: current net) and set
    /// information of a net, i.e., net serial number, minimun net
    /// width.
    ///@param[in] netName The net name
    ///@param[in] netSerial The serial number of the net
    ///@param[in] pinNumber Total pin number of the net
    ///@param[in] minWidth Minimun net width of the net
    virtual void 	beginAddANet (const char* netName,
                                  unsigned int netSerial,
                                  unsigned int pinNumber,
                                  unsigned int minWidth) = 0;
	
    ///@brief Add a pin to current net
    virtual void    addPin (unsigned int x,
                            unsigned int y,
                            unsigned int layer) = 0;

    ///@brief End of adding pin to current net and current net won't
    /// update anymore.
    ///@details You can remove the net if it contains only one pin or 
    /// over 1000 pins or something else at this function.
    virtual void    endAddANet () = 0;

    ///@brief End of parsing the input file
    virtual void    endBuild () = 0;
};
#endif //INC_BUILDER_H
