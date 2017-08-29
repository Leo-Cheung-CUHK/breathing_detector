# Copyright 2011 Free Software Foundation, Inc.
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

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT rawofdm_sources)
	MESSAGE(STATUS "No C++ sources... skipping swig/")
	return()
endif(NOT rawofdm_sources)

########################################################################
# Include swig generation macros
########################################################################
find_package(SWIG)
find_package(PythonLibs 2)
if(NOT SWIG_FOUND OR NOT PYTHONLIBS_FOUND)
    return()
endif()
include(GrSwig)
include(GrPython)

########################################################################
# Setup swig generation
########################################################################
foreach(incdir ${GNURADIO_RUNTIME_INCLUDE_DIRS})
    list(APPEND GR_SWIG_INCLUDE_DIRS ${incdir}/gnuradio/swig)
endforeach(incdir)

set(GR_SWIG_LIBRARIES rawofdm)

set(GNURADIO_RUNTIME_SWIG_INCLUDE_DIRS
    ${GNURADIO_RUNTIME_INCLUDE_DIRS}/gnuradio/swig
)

set(GR_SWIG_INCLUDE_DIRS
    ${GR_BLOCKS_INCLUDE_DIRS}
    ${GNURADIO_RUNTIME_INCLUDE_DIRS}
    ${GNURADIO_RUNTIME_SWIG_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/ofdm
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/qam
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/util
)

message(STATUS "swig >>> : ${GNURADIO_RUNTIME_INCLUDE_DIRS}")
message(STATUS "swig >>> : ${GNURADIO_RUNTIME_SWIG_INCLUDE_DIRS}")

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_swig.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/raw_swig.py
@ONLY)

# We split up the swig files into multiple sections to minimize the
# memory overhead. If a .i file grows too large, create a new file
# named 'blocks_swigN.i' and add it to this list.
#
# Also add the line "from swig_blocksN import *" line to
# blocks_swig.py.in.
set(GR_SWIG_BLOCK_IFILES
  raw_ofdm
  raw_util
  raw_qam
)

foreach(swigfile ${GR_SWIG_BLOCK_IFILES})
  GR_SWIG_MAKE(${swigfile} ${swigfile}.i)
  
  GR_SWIG_INSTALL(
    TARGETS ${swigfile}
    DESTINATION ${GR_PYTHON_DIR}/raw
  )

  list(APPEND SWIGFILES ${swigfile}.i)
endforeach(swigfile)

install(
    FILES
    ${SWIGFILES}
    DESTINATION ${GR_INCLUDE_DIR}/raw/swig
)

# Install the Python file that pulls in the swig built files.
GR_PYTHON_INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/raw_swig.py
    DESTINATION ${GR_PYTHON_DIR}/raw
)
