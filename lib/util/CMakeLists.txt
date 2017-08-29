# Copyright 2010-2011 Free Software Foundation, Inc.
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
# This file included, use CMake directory variables
########################################################################
set(rawofdm_util_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_peak_detector_fb.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_peak_detector2_fb.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_message_sink.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_message.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_msg_queue.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_pnc_frequency_modulator_fc.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_message_source.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_regenerate_peak2.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_regenerate_peak3.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_divide_ff.cc
)

########################################################################
# Setup sources and includes
########################################################################
list(APPEND rawofdm_sources ${rawofdm_util_sources})

install( FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_peak_detector_fb.h
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_peak_detector2_fb.h
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_message_sink.h
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_msg_queue.h
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_pnc_frequency_modulator_fc.h
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_message_source.h
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_regenerate_peak2.h
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_regenerate_peak3.h
    ${CMAKE_CURRENT_SOURCE_DIR}/raw_divide_ff.h
    DESTINATION ${GR_INCLUDE_DIR}/raw
)

########################################################################
# Register unit tests
########################################################################
#if(ENABLE_TESTING)
#add_executable(gr_core_rstest
#    ${gr_core_rs_sources}
#    ${CMAKE_CURRENT_SOURCE_DIR}/rstest.c
#    ${CMAKE_CURRENT_SOURCE_DIR}/exercise.c
#)
#add_test(gr-core-reed-solomon-test gr_core_rstest)
#endif(ENABLE_TESTING)

########################################################################
# Setup executables
########################################################################
#add_executable(rscode_example example.cc)
#target_link_libraries(rscode_example gnuradio-core ${Boost_LIBRARIES})
