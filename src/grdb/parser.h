// File: grdb/parser.h
// Brief: Parser for parsing test case of ISPD'07 and ISPD'98
// Author: Yen-Jung Chang
// $Date: 2007-11-26 12:48:59 +0800 (Mon, 26 Nov 2007) $
// $Revision: 12 $

#ifndef INC_PARSER_H
#define INC_PARSER_H

#include "builder.h"
#include "../misc/filehandler.h"

using Jm::FileHandler;

/**
 * @brief Glboal Router Parser insterface
 */
class GRParser {
public:
    virtual ~GRParser();

    /// @brief Parse file with Builder
    /// @param[in] builder Callback functions for parser
    virtual void parse(Builder* builder) = 0;
};

/** 
 * @brief Global Router Parser for parsing ISPD'07 test case
 */
class Parser07: public GRParser {
    string fname_;      ///< File name
    string delims_;
    FileHandler fh_;    ///< File Handler
    Builder* builder_;    ///< Data stucture builder
public:
    Parser07(const char* fname, FileHandler::FileType ftype);
    virtual ~Parser07();
    virtual void parse(Builder* builder);

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
class Parser98: public GRParser {
    string fname_;      ///< File name
    string delims_;
    FileHandler fh_;    ///< File Handler
    Builder* builder_;  ///< Data stucture builder
public:
    Parser98(const char* fname, FileHandler::FileType ftype);
    virtual ~Parser98();

    virtual void parse(Builder* builder);

private:
    /// Parse the information of routing layers, tiles
    void parseRoutingRegion();

    /// Parse the information of nets
    void parseNets();

    /// Parse information of one net
    void parseANet();
};

//======= Inline Functions =======
inline Parser07::Parser07(const char* fname, FileHandler::FileType ftype) :
        fname_(fname), delims_(" \t\n"), fh_(fname, ftype) {
}

inline Parser98::Parser98(const char* fname, FileHandler::FileType ftype) :
        fname_(fname), delims_(" \t\n"), fh_(fname, ftype) {
}
#endif //INC_PARSER_H
