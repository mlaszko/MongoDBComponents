# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find OpenCV library files
FIND_PACKAGE( OpenCV REQUIRED )
#FIND_PACKAGE( MongoDB REQUIRED)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Create an executable file from sources:

set( LIBS_TO_LINK
    ${Boost_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS} 
    ${DisCODe_LIBRARIES} 
    ${MongoDB_LIBRARIES}
)

ADD_LIBRARY(MongoDBExporter SHARED ${files} )

TARGET_LINK_LIBRARIES(MongoDBExporter pthread ${LIBS_TO_LINK} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})


INSTALL_COMPONENT(MongoDBExporter)
