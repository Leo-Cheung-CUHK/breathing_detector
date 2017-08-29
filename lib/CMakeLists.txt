# Copyright 2011-2013 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

include(GrMiscUtils)

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

# *** For Viterbi Decoder, change these depending on CPU Instruction Set ***
#set(CMAKE_C_FLAGS "-std=c99 -msse4.2")
# -mssse3
set(CMAKE_C_FLAGS "-std=c99")
#set(CMAKE_C_FLAGS "-std=c99 -msse4.2 -Ofast")
#set(CMAKE_CXX_FLAGS "-mssse4.2")

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

GR_INCLUDE_SUBDIRECTORY(spiral)
GR_INCLUDE_SUBDIRECTORY(util)
GR_INCLUDE_SUBDIRECTORY(qam)
GR_INCLUDE_SUBDIRECTORY(ofdm)

include_directories(
    ofdm
    qam
    spiral
    util
)

message(STATUS "")
message(STATUS "rawofdm_sources >>>>>> " ${rawofdm_sources})



set(rawofdm_sources "${rawofdm_sources}" PARENT_SCOPE)
if(NOT rawofdm_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT rawofdm_sources)

if(ENABLE_GR_CTRLPORT)
  ADD_DEFINITIONS(-DGR_CTRLPORT)
endif(ENABLE_GR_CTRLPORT)

add_library(rawofdm SHARED ${rawofdm_sources})
target_link_libraries(rawofdm ${Boost_LIBRARIES} ${GNURADIO_ALL_LIBRARIES} m)
set_target_properties(rawofdm PROPERTIES DEFINE_SYMBOL "rawofdm_EXPORTS" VERSION 1.3.7 SOVERSION 1.3)

########################################################################
# Install built library files
########################################################################
install(TARGETS rawofdm
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so/.dylib file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)
