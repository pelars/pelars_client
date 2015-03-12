cmake_minimum_required(VERSION 2.8)
Project (sensor_manager CXX)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/modules")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE Release) 
set(LIBWEBSOCKETPP_LIBRARIES cppnetlib-client-connections cppnetlib-server-parsers cppnetlib-uri)
set(PATHS /usr/local/lib /usr/lib )


set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(OpenCV REQUIRED)
find_package(USB REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Boost COMPONENTS system filesystem thread random REQUIRED)
find_package(Freenect REQUIRED)
find_package(Websocketpp REQUIRED)

find_library(JSON_LIBRARY NAMES jsoncpp)
#http://cpp-netlib.org/
find_library(CPPNET_CLIENT cppnetlib-client-connections ${PATHS} ) 
find_library(CPPNET_URI cppnetlib-uri)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${INCLUDE})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${LIBWEBSOCKETS_INCLUDE_DIR})

add_executable(sensor_manager ${SRC}/main.cpp ${SRC}/opt.cpp ${SRC}/linemod.cpp ${SRC}/kinect_grabber.cpp ${SRC}/data_writer.cpp ${SRC}/mongoose.cpp)
add_executable(freenect-camshare ${SRC}/camshare.cpp)

link_libraries(LIBWEBSOCKETS_LIBRARY_DIR)

target_link_libraries(sensor_manager ${OpenCV_LIBS} ${Boost_LIBRARIES} ${JSON_LIBRARY} ${LIBWEBSOCKETPP_LIBRARIES} ${OPENSSL_LIBRARIES})
if(UNIX AND NOT APPLE)
    target_link_libraries(freenect-camshare freenect rt)
else()
	target_link_libraries(freenect-camshare freenect)
endif()
