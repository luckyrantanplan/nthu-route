// File: misc/filehandler.h
// Brief: Provide basic operations for processing files
// Author: Yen-Jung Chang
// $Date: 2007-11-26 12:48:59 +0800 (Mon, 26 Nov 2007) $
// $Revision: 12 $

#ifndef INC_FILEHANDLER_H
#define INC_FILEHANDLER_H

#include <zlib.h>
#include <string>

using std::string;

namespace Jm {

/**
@brief Handle file basic operations. 
**/
class FileHandler{
  public:
    enum FileType {
        AutoFileType,       ///< Auto detect file mode. *.gz => FileType = Gzip
        NormalFileType,     ///< Umcompressed file
        GzipFileType        ///< Gzipped file
    };
    enum AccessMode {
        ReadAccessMode,     ///< Read Only
        WriteAccessMode     ///< Write permisson is granded
    };

  private:
    string      fname_;     ///< File name
    bool        isOpen_;    ///< true => opened
    class File* file_;      ///< File instance

  public:
                FileHandler(const char* fname, FileType ftype);
                ~FileHandler();

	/// Open file with specifed access mode
    bool        open(AccessMode accMode);

	/// Close file
    int         close();

	/// @brief Read characters from file until length-1 characters are read or a new line
    /// character is read and transferred to buf, or an end-of-file condition is 
    /// encountered.  The string is then terminated with a null character.
	/// @param[in]  length The length of input string buffer
	/// @param[out] buffer The read string from file will put here
    /// @return char* to buffer, or NULL in case of error.
    char*		getline(char* buffer, int length);

    /// @brief Skip one line from file
    void        skipline();

	/// @brief Write a line to file
	/// @param[in] buffer Write the string into file
    int			writeline(const char* buffer);
};

/**
@brief Interface of file. Here provide some basic operations for a file, 
you should implement the detail functions in other class. 
It should also cover the details of compressed or decompressed file.
**/
class File{
  public:
					File();
    virtual			~File();

	/// Open file with specified access mode
    virtual bool	open(const char* fname,
						 FileHandler::AccessMode accMode) = 0;
	
	/// Close file
    virtual int		close() = 0;

	/// @brief Read characters from file until length-1 characters are read or a new line
    /// character is read and transferred to buf, or an end-of-file condition is 
    /// encountered.  The string is then terminated with a null character.
	/// @param[in]  length The length of input string buffer
	/// @param[out] buffer The read string from file will put here
    /// @return char* to buffer, or NULL in case of error.
    virtual char*	getline(char* buffer, int length) = 0;

    /// @brief Skip a line from file
    virtual void    skipline() = 0;

	/// @brief Write a line to file
	/// @param[in] buffer Write the string into file
    virtual int		writeline(const char* buffer) = 0;
};

/**
@brief Implementation for reading gzipped file.
**/
class GzipFile: public File {
    gzFile      gzf_;       ///< zlib file handler
  public:
                    GzipFile();
    virtual         ~GzipFile();
    virtual bool    open(const char* fname, FileHandler::AccessMode accMode);
    virtual int     close();
    virtual char*   getline(char* buffer, int length);
    virtual void    skipline();
    virtual int     writeline(const char* buffer);
};

/**
@brief normalFile: Implementation for reading normal file.
**/
class NormalFile: public File {
    FILE*       fd_;   ///< File descripter
  public:
                    NormalFile();
    virtual         ~NormalFile();
    virtual bool    open(const char* fname, FileHandler::AccessMode accMode);
    virtual int     close();
    virtual char*   getline(char* buffer, int length);
    virtual void    skipline();
    virtual int     writeline(const char* buffer);
};

//========== Inline Functions ============
inline
bool FileHandler::open(AccessMode accMode)
{
    isOpen_ = true;
    return file_->open(fname_.c_str(), accMode);
}

inline
int FileHandler::close()
{
    if(isOpen_ == false) return -1;
    isOpen_ = false;
    return file_->close();
}

inline
char* FileHandler::getline(char* buffer, int length)
{
    if(isOpen_) {
        return file_->getline(buffer, length);
    } else {
        return NULL;
    }
}

inline
void FileHandler::skipline()
{
    if(isOpen_) file_->skipline();
}

inline
int FileHandler::writeline(const char* buffer)
{
    if(isOpen_) return file_->writeline(buffer);
    else return -1;
}

} //end of namespace
#endif //INC_FILEHANDLER_H
