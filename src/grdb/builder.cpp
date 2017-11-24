//  File: grdb/builder.cpp
//  Brief: Provide an interface for parser to insert data into database
//  Author: Yen-Jung Chang
//  $Date: 2007-11-26 17:18:13 +0800 (Mon, 26 Nov 2007) $
//  $Revision: 13 $

#include "builder.h"
#include "iostream"

using namespace std;

Builder::~Builder () {}

void Builder::setGrid (unsigned int,                //x
                       unsigned int,                //y
                       unsigned int)                //layerNumber
{}

void Builder::setVerticalCapacity (unsigned int,    //layerId
                                   unsigned int)    //capacity
{}

void Builder::setHorizontalCapacity (unsigned int,  //layerId
                                     unsigned int)  //capacity
{}

void Builder::setNetNumber (unsigned int)           //netNumber
{}

void Builder::adjustEdgeCapacity (unsigned int,     //x1
                                  unsigned int,     //y1
                                  unsigned int,     //z1
                                  unsigned int,     //x2
                                  unsigned int,     //y2
                                  unsigned int,     //z2
                                  unsigned int)     //capacity
{}

void Builder::setLayerMinimumWidth (unsigned int,   //layerId,
                                    unsigned int)   //width
{}

void Builder::setLayerMinimumSpacing (unsigned int, //layerId
                                      unsigned int) //spacing
{}

void Builder::setViaSpacing (unsigned int,          //layerId
                             unsigned int)          //viaSpacing
{}

void Builder::setTileTransformInformation (unsigned int,    //llx
                                           unsigned int,    //lly 
                                           unsigned int,    //tWidth
                                           unsigned int)    //tHeight
{}

void Builder::beginAddANet (const char*,            //netName
                            unsigned int,           //netSerial
                            unsigned int,           //pinNumber
                            unsigned int)           //minWidth
{}

void Builder::addPin (unsigned int,                 //x
                      unsigned int,                 //y
                      unsigned int)                 //layer
{}

void Builder::endAddANet ()
{}

void Builder::endBuild ()
{}
