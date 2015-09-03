cmake_minimum_required(VERSION 2.8)
Project (sensor_manager CXX)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/modules")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE Release) 

set(CMAKE_PREFIX_PATH /prefix/)
set(SRC                ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE            ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include(${CMAKE_ROOT}/Modules/FindPkgConfig.cmake)

find_package(OpenCV REQUIRED)
find_package(USB REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Boost COMPONENTS system filesystem thread random REQUIRED)
find_package(Freenect REQUIRED)
find_package(Websocketpp REQUIRED)
find_package(PkgConfig REQUIRED)
#find_package(FFmpeg REQUIRED)

find_library(JSON_LIBRARY  NAMES jsoncpp)
find_library(CPPNET_CLIENT NAMES cppnetlib-client-connections) 
find_library(CPPNET_PARSER NAMES cppnetlib-server-parsers)
find_library(CPPNET_URI    NAMES cppnetlib-uri )
find_library(FREENECT      NAMES freenect)

find_library(TINYXML2        NAMES tinyxml2)
find_library(X264            NAMES x264) 
find_library(SWSCALE         NAMES swscale)
find_library(AVFORMAT        NAMES avformat)
find_library(AVUTIL          NAMES avutil)
find_library(AVCODEC         NAMES avcodec)
find_library(AVFILTER        NAMES avfilter)
find_library(AVDEVICE        NAMES avdevice)
find_library(CURL            NAMES curl)
find_library(ARUCO           NAMES aruco)
find_library(GSTREAMER       NAMES gstreamer-1.0)
find_library(GSTREAMER_O     NAMES gobject-2.0)
find_library(GSTREAMER_A     NAMES gstapp-1.0)
find_library(GLIB            NAMES glib-2.0)


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${INCLUDE})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${x264_INCLUDE_DIRS})

# Set CMAKE_C_FLAGS variable with info from pkg-util
execute_process(COMMAND pkg-config --cflags gstreamer-1.0
                OUTPUT_VARIABLE CMAKE_C_FLAGS)
string(REPLACE "\n" "" CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
message("CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")

add_executable(sensor_manager   ${SRC}/main.cpp 
								${SRC}/session_manager.cpp 
								${SRC}/face_detector.cpp
								${SRC}/linemod.cpp 
								${SRC}/kinect_grabber.cpp 
								${SRC}/data_writer.cpp 
								${SRC}/mongoose.cpp 
								${SRC}/x264decoder.cpp 
								${SRC}/x264encoder.cpp
								${SRC}/sse_handler.cpp 
								${SRC}/xn16zdec.cpp
								${SRC}/hand_detector.cpp
								${SRC}/gstreamer_grabber.cpp
								${SRC}/opt.cpp
								${SRC}/key_logger.cpp)

add_executable(freenect-camshare ${SRC}/camshare.cpp)

target_link_libraries(sensor_manager 	${OpenCV_LIBS} 
										${TINYXML2} 
										${Boost_LIBRARIES} 
										${JSON_LIBRARY}
										${CPPNET_CLIENT} 
										${CPPNET_PARSER}
										${CPPNET_URI}
										${OPENSSL_LIBRARIES} 
										${ZMQ_LIBRARY} 
										${X264} 
										${SWSCALE} 
										${AVFORMAT}
										${AVUTIL} 
										${AVCODEC} 
										${AVFILTER} 
										${AVDEVICE}
										${CURL}
										${ARUCO}
										${GSTREAMER}
										${GSTREAMER_O}
										${GSTREAMER_A}
										${GLIB}																
                                        rt )

set_target_properties(sensor_manager PROPERTIES COMPILE_FLAGS ${CMAKE_C_FLAGS})                                        
target_link_libraries(freenect-camshare ${FREENECT} rt) 


