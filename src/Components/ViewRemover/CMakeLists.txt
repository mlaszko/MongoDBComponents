# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find OpenCV library files
FIND_PACKAGE( OpenCV REQUIRED )

INCLUDE_DIRECTORIES(/home/lzmuda/mongo-client-install/include)
INCLUDE_DIRECTORIES(/home/lzmuda/DisCODe/build/inst/include/Base)
INCLUDE_DIRECTORIES(/home/lzmuda/DisCODe/build/inst/include/Client)
INCLUDE_DIRECTORIES(/home/lzmuda/DisCODe/build/inst/include/Common)
LINK_DIRECTORIES(/home/lzmuda/mongo-client-install/lib)


# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Create an executable file from sources:


set( LIBS_TO_LINK
    ${Boost_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS} 
    ${DisCODe_LIBRARIES} 
    libmongoclient.a
)

ADD_LIBRARY(ViewRemover SHARED ${files} )

TARGET_LINK_LIBRARIES(ViewRemover pthread ${LIBS_TO_LINK} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

INSTALL_COMPONENT(ViewRemover)



