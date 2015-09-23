cmake_minimum_required(VERSION 2.8)
Project (sensor_manager CXX)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/modules")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE Release) 

set(CMAKE_PREFIX_PATH /prefix)
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
find_package(Websocketpp REQUIRED)
find_package(Gstreamer-1.0 REQUIRED)
find_package(Freenect REQUIRED)
find_package(Websocketpp REQUIRED)
find_package(FFmpeg REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(JsonCpp REQUIRED)
find_package(x264 REQUIRED)
find_package(Cppnetlib REQUIRED)
find_package(CURL REQUIRED)
find_package(Aruco REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(FFmpeg REQUIRED)
find_package(PCL 1.7 REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

set(CPPNET_CLIENT ${CPPNETLIB_LIBRARIES})
set(CPPNET_PARSER ${CPPNETLIB_LIBRARIES})
set(CPPNET_URI ${CPPNETLIB_LIBRARIES})
set(FREENECT ${FREENECT_LIBRARIES})
set(ARUCO ${ARUCO_LIBRARIES})
set(CURL ${CURL_LIBRARIES})
set(AVFILTER ${FFMPEG_LIBRARIES})
set(JSON_LIBRARY ${JSONCPP_LIBRARY})
set(X264 ${X264_LIBRARIES})
set(TINYXML2 ${TINYXML2_LIBRARY})

#Should be fixed in the future
find_library(GSTREAMER_O     NAMES gobject-2.0)
find_library(GSTREAMER       NAMES gstreamer-1.0)
find_library(GSTREAMER_A     NAMES gstapp-1.0)
find_library(GLIB            NAMES glib-2.0)
find_library(FREENECT2 NAMES freenect2)
set(GSTREAMER_ALL ${GSTREAMER} ${GSTREAMER_O} ${GSTREAMER_A} ${GLIB})
find_library(PORTAUDIO portaudio)

include_directories(${JSONCPP_INCLUDE_DIR})
include_directories(${FFMPEG_INCLUDE_DIRS})
include_directories(${GSTREAMER_INCLUDE_DIRS})
include_directories(${FREENECT_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${INCLUDE})
include_directories(${GLFW_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

set(LINUXEXTRA rt boost_random)
add_definitions(${PCL_DEFINITIONS})

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
								${SRC}/key_logger.cpp
								${SRC}/gstreamer_grabber2.cpp
								${SRC}/ide_handler.cpp
								${SRC}/audio_detector.cpp
								)

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
                                        ${LINUXEXTRA} 
										${GSTREAMER_ALL}
										${FFMPEG_LIBRARIES}
										${FREENECT2}
										${GLFW_LIBRARIES}
										${PCL_LIBRARIES}
										${PORTAUDIO}
										)

target_link_libraries(freenect-camshare ${FREENECT} rt) 


