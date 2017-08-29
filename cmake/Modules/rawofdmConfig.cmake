INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_RAWOFDM rawofdm)

FIND_PATH(
    RAWOFDM_INCLUDE_DIRS
    NAMES rawofdm_api.h
    HINTS $ENV{RAWOFDM_DIR}/include
        ${PC_RAWOFDM_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    RAWOFDM_LIBRARIES
    NAMES rawofdm
    HINTS $ENV{RAWOFDM_DIR}/lib
        ${PC_RAWOFDM_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(RAWOFDM DEFAULT_MSG RAWOFDM_LIBRARIES RAWOFDM_INCLUDE_DIRS)
MARK_AS_ADVANCED(RAWOFDM_LIBRARIES RAWOFDM_INCLUDE_DIRS)

