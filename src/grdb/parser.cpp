// File: grdb/parser.cpp
// Brief: Parser for parsing test case of ISPD'07 and ISPD'98
// Author: Yen-Jung Chang
// $Date: 2007-11-26 17:18:13 +0800 (Mon, 26 Nov 2007) $
// $Revision: 13 $

#include "parser.h"

#include <array>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "RoutingRegion.h"

namespace NTHUR {

#define MAX_STRING_BUFER_LENGTH 512
#define MAX_PIN 1000

//====== GRParser =========

//====== Parser07 =========
//{{{
// virtual
Parser07::~Parser07() {
}

// virtual
RoutingRegion Parser07::parse() {

    // Begin to parse file
    if (!fh_.open(FileHandler::ReadAccessMode)) {
        std::cerr << "Error opening test case." << std::endl;
        abort();
    }
    RoutingRegion builder(parseRoutingRegion());
    parseNets(builder);
    adjustCapacity(builder);
    return builder;
}

// virtual
RoutingRegion Parser07::parseRoutingRegion() {
    // buffer for strtok()
    std::array<char, MAX_STRING_BUFER_LENGTH> stringBuffer;

    // Get grid size and layer number
    fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
    // get grid size
    atoi(strtok(stringBuffer.data(), delims_.c_str()));  // "grid" string
    int x = atoi(strtok(NULL, delims_.c_str()));
    int y = atoi(strtok(NULL, delims_.c_str()));
    int layerNumber = atoi(strtok(NULL, delims_.c_str()));
    RoutingRegion builder_(x, y, layerNumber);

    // Set vertical capacity
    fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
    strtok(stringBuffer.data(), delims_.c_str());      // "vertical"
    strtok(NULL, delims_.c_str());              // "capacity"
    // for each layer, set the vertical capacity
    for (int i = 0; i < layerNumber; ++i) {
        int capacity = atoi(strtok(NULL, delims_.c_str()));
        builder_.setVerticalCapacity(i, capacity);
    }

    // Set horizontal capacity
    fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
    strtok(stringBuffer.data(), delims_.c_str());      // "horizontal"
    strtok(NULL, delims_.c_str());              // "capacity"
    // for each layer, set the horizontal capacity
    for (int i = 0; i < layerNumber; ++i) {
        int capacity = atoi(strtok(NULL, delims_.c_str()));
        builder_.setHorizontalCapacity(i, capacity);
    }

    // Set minimum width
    fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
    strtok(stringBuffer.data(), delims_.c_str());      // "minimum"
    strtok(NULL, delims_.c_str());              // "width"
    // for each layer, set the minimum width
    for (int i = 0; i < layerNumber; ++i) {
        int width = atoi(strtok(NULL, delims_.c_str()));
        builder_.setLayerMinimumWidth(i, width);
    }

    // Set minimum spacing
    fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
    strtok(stringBuffer.data(), delims_.c_str());        // "minimum"
    strtok(NULL, delims_.c_str());              // "spacing"
    // for each layer, set the minimum spacing
    for (int i = 0; i < layerNumber; ++i) {
        int spacing = atoi(strtok(NULL, delims_.c_str()));
        builder_.setLayerMinimumSpacing(i, spacing);
    }

    // Set via spacing
    fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
    strtok(stringBuffer.data(), delims_.c_str());        // "via"
    strtok(NULL, delims_.c_str());              // "spacing"
    // for each layer, set the via spacing
    for (int i = 0; i < layerNumber; ++i) {
        int spacing = atoi(strtok(NULL, delims_.c_str()));
        builder_.setViaSpacing(i, spacing);
    }

    // Set tile transformation information
    fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
    int llx = atoi(strtok(stringBuffer.data(), delims_.c_str()));
    int lly = atoi(strtok(NULL, delims_.c_str()));
    int tileWidth = atoi(strtok(NULL, delims_.c_str()));
    int tileHeight = atoi(strtok(NULL, delims_.c_str()));
    builder_.setTileTransformInformation(llx, lly, tileWidth, tileHeight);

    return builder_;
}

void Parser07::parseNets(RoutingRegion& builder_) {
    // buffer for strtok()
    std::array<char, MAX_STRING_BUFER_LENGTH> stringBuffer;

    // get rid of empty line
    do {
        memset(stringBuffer.data(), '\0', MAX_STRING_BUFER_LENGTH);
        fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
    } while (!isalnum(stringBuffer[0]));
    // Get total net number
    strtok(stringBuffer.data(), delims_.c_str());      // "num"
    strtok(NULL, delims_.c_str());              // "net"
    int netNumber = atoi(strtok(NULL, delims_.c_str()));    // read total net number
    builder_.setNetNumber(netNumber);

    for (int i = 0; i < netNumber; ++i) {
        parseANet(builder_);
    }

}

void Parser07::parseANet(RoutingRegion& builder_) {
    // buffer for strtok()
    std::array<char, MAX_STRING_BUFER_LENGTH> stringBuffer;

    // Get net information
    fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
    std::string netName = strtok(stringBuffer.data(), delims_.c_str());
    int netSerial = atoi(strtok(NULL, delims_.c_str()));    // read net serial number
    int pinNumber = atoi(strtok(NULL, delims_.c_str()));    // read pin number
    int minWidth = atoi(strtok(NULL, delims_.c_str())); // read net minmum width

    if (pinNumber <= MAX_PIN) {   // pin# > 1000 is a special net, we can skip it
        builder_.beginAddANet(netName.c_str(), netSerial, pinNumber, minWidth);  // Add a net to DB
        // reading pin information of a net
        for (int j = 0; j < pinNumber; ++j) {
            fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
            int x = atoi(strtok(stringBuffer.data(), delims_.c_str()));  // x of pin
            int y = atoi(strtok(NULL, delims_.c_str()));        // y of pin
            int layer = atoi(strtok(NULL, delims_.c_str()));    // layer of pin
            builder_.addPin(x, y, layer - 1);  //Add pin to a net
        }
        builder_.endAddANet();                             // end of reading a net
    } else {
        // skip reading net information with >1000 pins
        for (int j = 0; j < pinNumber; ++j) {
            fh_.skipline();
        }
    }

}

void Parser07::adjustCapacity(RoutingRegion& builder_) {
    // buffer for strtok()
    std::array<char, MAX_STRING_BUFER_LENGTH> stringBuffer;

    // get rid of empty line
    do {
        memset(stringBuffer.data(), '\0', MAX_STRING_BUFER_LENGTH);
        fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
    } while (!isalnum(stringBuffer[0]));

    // Get adjustment number
    // get the total edge adjusting number
    int adjustNumber = atoi(strtok(stringBuffer.data(), delims_.c_str()));

    for (int i = 0; i < adjustNumber; ++i) {
        fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
        // reading source gCell
        int x1 = atoi(strtok(stringBuffer.data(), delims_.c_str()));
        int y1 = atoi(strtok(NULL, delims_.c_str()));
        int z1 = atoi(strtok(NULL, delims_.c_str()));
        // reading sink gCell
        int x2 = atoi(strtok(NULL, delims_.c_str()));
        int y2 = atoi(strtok(NULL, delims_.c_str()));
        int z2 = atoi(strtok(NULL, delims_.c_str()));
        // reading the new capacity
        int capacity = atoi(strtok(NULL, delims_.c_str()));
        builder_.adjustEdgeCapacity(x1, y1, z1 - 1, x2, y2, z2 - 1, capacity);
    }

}
//}}}

//====== Parser98 =======
//{{{
Parser98::~Parser98() {
}

RoutingRegion Parser98::parse() {

    // Begin to parse file
    if (!fh_.open(FileHandler::ReadAccessMode)) {
        std::cerr << "Error opening test case." << std::endl;
        abort();
    }
    RoutingRegion builder(parseRoutingRegion());
    parseNets(builder);
    return builder;

}

RoutingRegion Parser98::parseRoutingRegion() {
    std::array<char, MAX_STRING_BUFER_LENGTH> stringBuffer; // buffer for strtok()

    // Get grid size and layer number
    fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
    // get grid size
    strtok(stringBuffer.data(), delims_.c_str());        // "grid" string
    int x = atoi(strtok(NULL, delims_.c_str()));
    int y = atoi(strtok(NULL, delims_.c_str()));
    RoutingRegion builder_(x, y, 1);                     // All test cases in ISPD'98 are single layer

    // Set vertical capacity
    fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
    strtok(stringBuffer.data(), delims_.c_str());        // "vertical"
    strtok(NULL, delims_.c_str());              // "capacity"
    // set the vertical capacity
    int capacity = atoi(strtok(NULL, delims_.c_str()));
    builder_.setVerticalCapacity(0, capacity);

    // Set horizontal capacity
    fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
    strtok(stringBuffer.data(), delims_.c_str());        // "horizontal"
    strtok(NULL, delims_.c_str());              // "capacity"
    // set the horizontal capacity
    capacity = atoi(strtok(NULL, delims_.c_str()));
    builder_.setHorizontalCapacity(0, capacity);

    // Set minimum width
    int width = 1;
    builder_.setLayerMinimumWidth(0, width);

    // Set minimum spacing
    int spacing = 0;
    builder_.setLayerMinimumSpacing(0, spacing);

    // Set via spacing
    int viaSpacing = 0;
    builder_.setViaSpacing(0, viaSpacing);

    // Set tile transformation information
    int llx = 0;
    int lly = 0;
    int tileWidth = 1;
    int tileHeight = 1;
    builder_.setTileTransformInformation(llx, lly, tileWidth, tileHeight);
    return builder_;
}

void Parser98::parseNets(RoutingRegion& builder_) {
    // buffer for strtok()
    std::array<char, MAX_STRING_BUFER_LENGTH> stringBuffer;

    // get rid of empty line
    do {
        memset(stringBuffer.data(), '\0', MAX_STRING_BUFER_LENGTH);
        fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
    } while (!isalnum(stringBuffer[0]));
    // Get total net number
    strtok(stringBuffer.data(), delims_.c_str());      // "num"
    strtok(NULL, delims_.c_str());              // "net"
    int netNumber = atoi(strtok(NULL, delims_.c_str()));
    builder_.setNetNumber(netNumber);

    for (int i = 0; i < netNumber; ++i) {
        parseANet(builder_);
    }

}

void Parser98::parseANet(RoutingRegion& builder_) {
    // buffer for strtok()
    std::array<char, MAX_STRING_BUFER_LENGTH> stringBuffer;
    // Get net information
    fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
    std::string netName = strtok(stringBuffer.data(), delims_.c_str());
    int netSerialNumber = atoi(strtok(NULL, delims_.c_str()));
    int pinNumber = atoi(strtok(NULL, delims_.c_str()));
    int minWidth = 1;

    if (pinNumber < MAX_PIN) {   // pin# > 1000 is a special net, we don't need to route it
        builder_.beginAddANet(netName.c_str(), netSerialNumber, pinNumber, minWidth);
        for (int j = 0; j < pinNumber; ++j) {
            fh_.getline(stringBuffer.data(), MAX_STRING_BUFER_LENGTH);
            int x = atoi(strtok(stringBuffer.data(), delims_.c_str()));
            int y = atoi(strtok(NULL, delims_.c_str()));
            builder_.addPin(x, y, 0);
        }
        builder_.endAddANet();                             // end of reading a net
    } else {
        for (int j = 0; j < pinNumber; ++j) {
            fh_.skipline();
        }
    }

}

} // namespace NTHUR
