# Copyright 2012 OSMOCOM Project
#
# This file is part of MiriSDR
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
# Setup library
########################################################################
add_library(mirisdr_shared SHARED
    libmirisdr.c
)

target_link_libraries(mirisdr_shared
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

set_target_properties(mirisdr_shared PROPERTIES DEFINE_SYMBOL "mirisdr_EXPORTS")
set_target_properties(mirisdr_shared PROPERTIES OUTPUT_NAME mirisdr)
set_target_properties(mirisdr_shared PROPERTIES SOVERSION ${MAJOR_VERSION})
set_target_properties(mirisdr_shared PROPERTIES VERSION ${LIBVER})

add_library(mirisdr_static STATIC
    libmirisdr.c
)

if(WIN32)
add_library(libgetopt_static STATIC
    getopt/getopt.c
)
endif()

target_link_libraries(mirisdr_static
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

set_property(TARGET mirisdr_static APPEND PROPERTY COMPILE_DEFINITIONS "mirisdr_STATIC" )

# the baseband NCO's sine and cosine
if(UNIX)
target_link_libraries(mirisdr_shared m)
target_link_libraries(mirisdr_static m)
endif()

if(NOT WIN32)
# Force same library filename for static and shared variants of the library
set_target_properties(mirisdr_static PROPERTIES OUTPUT_NAME mirisdr)
endif()

########################################################################
# Build utility
########################################################################
add_executable(miri_sdr miri_sdr.c)
add_executable(miri_fm miri_fm.c)
add_executable(miri_eeprom miri_eeprom.c)
add_executable(miri_test miri_test.c)
set(INSTALL_TARGETS mirisdr_shared mirisdr_static miri_sdr miri_fm miri_eeprom miri_test)

target_link_libraries(miri_sdr mirisdr_shared
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

target_link_libraries(miri_fm mirisdr_shared
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

target_link_libraries(miri_eeprom mirisdr_shared
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

target_link_libraries(miri_test mirisdr_shared
    ${LIBUSB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
if(MIRISDR_SOAPY)
    # the soapy group: the module from this build, else the installed one. Its own
    # library keeps SoapySDR's C++ compile options off the C sources
    add_library(miri_test_soapy STATIC miri_test_soapy.cpp)
    target_include_directories(miri_test_soapy PRIVATE ${SoapySDR_INCLUDE_DIRS})
    target_link_libraries(miri_test_soapy PRIVATE ${SoapySDR_LIBRARIES})
    target_link_libraries(miri_test miri_test_soapy)
    target_compile_definitions(miri_test PRIVATE MIRI_TEST_SOAPY
        MIRI_SOAPY_MODULE_BUILD="$<TARGET_FILE:mirisdrSupport>"
        MIRI_SOAPY_MODULE_INSTALL="${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}/SoapySDR/modules${SOAPY_SDR_ABI_VERSION}/$<TARGET_FILE_NAME:mirisdrSupport>")
    add_dependencies(miri_test mirisdrSupport)
endif()

if(UNIX)
target_link_libraries(miri_fm m)
target_link_libraries(miri_test m)
endif()

if(WIN32)
target_link_libraries(miri_sdr libgetopt_static)
target_link_libraries(miri_fm libgetopt_static)
set_property(TARGET miri_sdr APPEND PROPERTY COMPILE_DEFINITIONS "mirisdr_STATIC" )
set_property(TARGET miri_fm APPEND PROPERTY COMPILE_DEFINITIONS "mirisdr_STATIC" )
endif()

########################################################################
# Install built library files & utilities
########################################################################
install(TARGETS ${INSTALL_TARGETS}
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so/.dylib file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)
