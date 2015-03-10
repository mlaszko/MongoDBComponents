SET(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(.)

# ##############################################################################
# Find required libraries
# ##############################################################################

# Find Boost, at least ver. 1.41
FIND_PACKAGE(Boost 1.41.0 REQUIRED)
FIND_PACKAGE( OpenCV REQUIRED )
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)


FIND_PACKAGE(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

DISCODE_FIND_DCL(PCLCoreTypes)
DISCODE_FIND_DCL(SIFTObjectModel)

# Find Boost
find_package( Boost COMPONENTS thread system program_options filesystem  REQUIRED ) #whatever libs you need
include_directories(SYSTEM ${Boost_INCLUDE_DIR})


# # OpenCV library
# FIND_PACKAGE( OpenCV REQUIRED )

# Find another DCLs our one depends on
# Use macro: DISCODE_FIND_DCL(<DCL_NAME>)

# ##############################################################################
# Add source directories
# ##############################################################################


ADD_SUBDIRECTORY(Components)
ADD_SUBDIRECTORY(Types)

# Prepare config file to use from another DCLs
CONFIGURE_FILE(MongoDBConfig.cmake.in ${CMAKE_INSTALL_PREFIX}/MongoDBConfig.cmake @ONLY)
