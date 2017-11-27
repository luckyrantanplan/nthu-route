// File: grdb/parser.h
// Brief: Parser for parsing test case of ISPD'07 and ISPD'98
// Author: Yen-Jung Chang
// $Date: 2007-11-26 12:48:59 +0800 (Mon, 26 Nov 2007) $
// $Revision: 12 $

#ifndef INC_PARSER_H
#define INC_PARSER_H

#include <string>

#include "../misc/filehandler.h"

class Builder;
class RoutingRegion;

using Jm::FileHandler;

/** 
 * @brief Global Router Parser for parsing ISPD'07 test case
 */
class Parser07 {
    string fname_;      ///< File name
    string delims_;
    FileHandler fh_;    ///< File Handler
    RoutingRegion& builder_;    ///< Data stucture builder
public:
    Parser07(const std::string& fname, FileHandler::FileType ftype, RoutingRegion& builder);
    virtual ~Parser07();
    virtual void parse();

private:
    /// Parse information of routing layers, tiles
    void parseRoutingRegion();

    /// Parse information of nets
    void parseNets();

    /// Parse information of one net
    void parseANet();

    /// Parse information of adjustmenting edge capacity
    void adjustCapacity();
};

/**
 * @brief Global Router Parser for parsing ISPD'98 test case
 */
class Parser98 {
    string fname_;      ///< File name
    string delims_;
    FileHandler fh_;    ///< File Handler
    RoutingRegion& builder_;  ///< Data stucture builder
public:
    Parser98(const std::string& fname, FileHandler::FileType ftype, RoutingRegion& builder);
    virtual ~Parser98();

    virtual void parse();

private:
    /// Parse the information of routing layers, tiles
    void parseRoutingRegion();

    /// Parse the information of nets
    void parseNets();

    /// Parse information of one net
    void parseANet();
};

//======= Inline Functions =======
inline Parser07::Parser07(const std::string& fname, FileHandler::FileType ftype, RoutingRegion& builder) :
        fname_(fname), delims_(" \t\n"), fh_(fname.data(), ftype), builder_ { builder } {
}

inline Parser98::Parser98(const std::string& fname, FileHandler::FileType ftype, RoutingRegion& builder) :
        fname_(fname), delims_(" \t\n"), fh_(fname.data(), ftype), builder_ { builder } {
}
#endif //INC_PARSER_H
