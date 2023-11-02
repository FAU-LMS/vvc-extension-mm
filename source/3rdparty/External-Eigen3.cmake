message( "-- Setup external project - Eigen3" )

# Add the external project
ExternalProject_Add(Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.3.7
    SOURCE_DIR eigen3
    BINARY_DIR eigen3-build
    INSTALL_DIR eigen3-install
    CMAKE_GENERATOR ${gen}
    CMAKE_ARGS
    ${ep_common_args}
        -DCMAKE_INSTALL_PREFIX=../eigen3-install
)

# Define paths and add include directories to search path
set( Eigen3_INSTALL_PATH "${CMAKE_BINARY_DIR}/eigen3-install" )
set( Eigen3_INCLUDE_DIRS "${Eigen3_INSTALL_PATH}/include/eigen3" )
include_directories(BEFORE "${Eigen3_INCLUDE_DIRS}")
