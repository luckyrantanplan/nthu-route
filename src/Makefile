# src/Makefile -- top-level Makefile
#
# last modified:   21-Oct-2004  Thu  07:42
#
# Copyright (c) 2004 SoftJin Infotech Private Ltd.
# This software may be used only under the terms of the SoftJin
# Source License.  See the accompanying file LICENSE for details.

CXX   = g++

ZlibIncDir =
Zlibrary   = -lz
##  If your system does not have zlib 1.1.4 or above, download and
##  install it, and add -I and -L directives here, e.g.,
##  ZlibIncDir = -I/usr/local/include
##  Zlibrary   = -L/usr/local/lib -lz
##  (or just Zlibrary = /usr/local/lib/libz.a)


FlutelibIncDir = -I..
Flutelibrary   = 
##	Make sure you have point to the directory which contains flute
##  library

#Threadlibrary   = -lboost_thread -lpthread
##	Make sure you have point to the directory which contains thread
##  library

GDlibrary = -lgd
##	Make sure you have point to the directory which contains GD
##  library


DebugOptions = -O3 -DNDEBUG -static -funroll-loops -march=nocona -finline-functions
##  Add -DNDEBUG to remove the debugging assertions (unwise at this stage).
##  You might have to remove -g or -O2 if your C++ compiler cannot
##  handle both together.


WarnOptions = -W -Wall -Wno-missing-braces
##  Warning options.  Enable all warnings except for those about missing
##  braces in struct/array initializer lists.


IncDirs      = -I. $(ZlibIncDir) $(FlutelibIncDir)
CxxOptions   = $(WarnOptions) $(DebugOptions)

CXXFLAGS     = $(CxxOptions) $(IncDirs)
COMPILE_CXX  = $(CXX) $(CXXFLAGS)
LINK_CXX     = $(CXX) $(CXXFLAGS)


StandardGarbage = a.out core core.[0-9][0-9]* make.log xxx
##  Junk to be removed from each source directory by the 'clean' target.


#-----------------------------------------------------------------------
# You should not need to change anything below.


bindir  = ../bin
libdir  = ../lib
objdir  = ../obj
docdir	= ../doc
docconfig	= doxygen_config
##  Executables are put in $(bindir), libraries in $(libdir), and .o
##  files in a subdirectory of $(objdir) corresponding to the source
##  subdirectory.  $(bindir), $(libdir) and $(objdir) with its
##  subdirectories are part of the distribution and are not deleted by
##  the clean targets.  That is because I do not know how to make sure
##  that even with "make -j4" the directories always get created before
##  the main build starts.


$(objdir)/%.o:	%.cpp
	$(COMPILE_CXX) -c $< -o $@

%.cpp:   %.l
	$(FLEX) -o$@ $?


subdirs       = misc       grdb		   util       flute       router    
subdirs_all   = misc_all   grdb_all    util_all   flute_all   router_all    
subdirs_clean = misc_clean grdb_clean  util_clean flute_clean router_clean  


.PHONY:	   dirs tags all clean $(subdirs_all) $(subdirs_clean)
.DEFAULT:  all


all:	$(subdirs_all)

clean:	$(subdirs_clean)
	rm -f $(StandardGarbage)

dirs:
	mkdir -p $(objdir) $(libdir) $(bindir) $(docdir)
	for dir in $(subdirs);  do  mkdir -p $(objdir)/$$dir;  done

doc:
	rm -rf $(docdir); mkdir -p $(docdir); doxygen $(docconfig)

tags:
	ctags */*.cpp */*.h


##  Follow Peter Miller's advice in his paper "Recursive Make
##  Considered Harmful".  Load all the subdirectory dependencies and
##  build the whole package from this directory instead of cd'ing
##  to each subdirectory and running make there.
##
##  Each foo/Make-defs file defines targets foo_all and foo_clean.
##

include misc/Makefile.in
include grdb/Makefile.in
include util/Makefile.in
include flute/Makefile.in
include router/Makefile.in
