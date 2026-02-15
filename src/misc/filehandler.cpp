// File: misc/filehandler.cpp
// Brief: Provide basic operations for processing files
// Author: Yen-Jung Chang
// $Date: 2007-11-26 12:48:59 +0800 (Mon, 26 Nov 2007) $
// $Revision: 12 $

#include "filehandler.h"
#include <zlib.h>
#include <cstring>
#include <cassert>
#include <cstdio>

namespace NTHUR {

//The string buffer length used for skipping a line
//The value must >= 3
constexpr int MAX_BUFFER_LENGTH_FOR_SKIP_LINE = 128;

FileHandler::FileHandler(const char* fname, FileType ftype) :
        fname_(fname), isOpen_(false) {
    //File FileType is auto, then look at the file extension to get real FileType
    if (ftype == AutoFileType) {
        const char* ext = strrchr(fname, '.');
        if (ext != nullptr && (strcmp(ext, ".gz") == 0)) {
            ftype = GzipFileType;
        } else {
            ftype = NormalFileType;
        }
    }

    switch (ftype) {
    case NormalFileType:
        file_ = new NormalFile();
        break;
    case GzipFileType:
        file_ = new GzipFile();
        break;
    default:
        assert(false);
    }
}

FileHandler::~FileHandler() {
    delete file_;
}

File::File() = default;

File::~File() = default;

GzipFile::GzipFile() :
        gzf_(nullptr) {
}

GzipFile::~GzipFile() = default;

bool GzipFile::open(const char* fname, FileHandler::AccessMode accMode) {
    const char* gzMode = (accMode == FileHandler::ReadAccessMode) ? "rb" : "wb";
    gzf_ = gzopen(fname, gzMode);
    return gzf_ != NULL;
}

int GzipFile::close() {
    return gzclose(gzf_);
}

char* GzipFile::getline(char* charBuffer, int length) {
    return gzgets(gzf_, charBuffer, length);
}

void GzipFile::skipline() {
    static char buffer[MAX_BUFFER_LENGTH_FOR_SKIP_LINE];
    do {
        //make sure the (lenth-1)nd char is NULL char (for while loop)
        buffer[MAX_BUFFER_LENGTH_FOR_SKIP_LINE - 2] = '\0';
        gzgets(gzf_, buffer, MAX_BUFFER_LENGTH_FOR_SKIP_LINE);
        //while the (lenth-1)nd char is not NULL or new line, keep reading
    } while (buffer[MAX_BUFFER_LENGTH_FOR_SKIP_LINE - 2] != '\0' && buffer[MAX_BUFFER_LENGTH_FOR_SKIP_LINE - 2] != '\n');
}

int GzipFile::writeline(const char* buffer) {
    return gzputs(gzf_, buffer);
}

NormalFile::NormalFile() :
        fd_(nullptr) {
}

NormalFile::~NormalFile() = default;

bool NormalFile::open(const char* fname, FileHandler::AccessMode accMode) {
    const char* Mode = (accMode == FileHandler::ReadAccessMode) ? "r" : "w";
    fd_ = fopen(fname, Mode);
    return fd_ != NULL;
}

int NormalFile::close() {
    return fclose(fd_);
}

char* NormalFile::getline(char* charBuffer, int length) {
    return fgets(charBuffer, length, fd_);
}

void NormalFile::skipline() {
    static char buffer[MAX_BUFFER_LENGTH_FOR_SKIP_LINE];
    do {
        buffer[MAX_BUFFER_LENGTH_FOR_SKIP_LINE - 2] = '\0';
        if (fgets(buffer, MAX_BUFFER_LENGTH_FOR_SKIP_LINE, fd_) == nullptr) {
            break;
        }
    } while (buffer[MAX_BUFFER_LENGTH_FOR_SKIP_LINE - 2] != '\0' && buffer[MAX_BUFFER_LENGTH_FOR_SKIP_LINE - 2] != '\n');
}

int NormalFile::writeline(const char* buffer) {
    return fputs(buffer, fd_);
}
} // namespace NTHUR
