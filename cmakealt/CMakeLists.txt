cmake_minimum_required(VERSION 2.8)
Project (sensor_manager CXX)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "../${CMAKE_SOURCE_DIR}/modules")
set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -Wall")

set(CMAKE_BUILD_TYPE   Release) 

set(SRC                ${CMAKE_CURRENT_SOURCE_DIR}/../src)
set(INCLUDE            ${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${INCLUDE})

find_package(OpenMP)
if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include(${CMAKE_ROOT}/Modules/FindPkgConfig.cmake)

set(Boost_USE_MULTITHREADED ON) 

find_package(PkgConfig REQUIRED)
#1.58 is the first with ASIO
find_package(Boost 1.58 COMPONENTS random program_options REQUIRED)
link_directories(${Boost_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
find_package(cmakego REQUIRED)
usepackage(opencv gstreamer freenect x11 freenect2 tinyxml2 x264 ffmpeg  portaudio x264 tinyxml2 glfw zeromq)
usepackage(json libusb REQUIRED)
usepackage(cairo ffmpeg REQUIRED)
find_package(PCL 1.7 REQUIRED)
#TODO for cmakego
#openssl
#websocketpp
#cppnetlib
#curl
#qrencode
find_package(Websocketpp REQUIRED)
message(NOTE Cppnetlib has to be at least 0.13 using boost::asio)
find_package(Cppnetlib REQUIRED)

#Internal ASIO ISSUE
#add_definitions(-DASIO_HEADER_ONLY)
#add_definitions(-DBOOST_NETWORK_ENABLE_HTTPS)
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../deps/asio/asio/include)

include_directories(${Boost_LIBRARY_DIRS})

if(TARGET p::freenect2)
add_definitions(-DHAS_FREENECT2)
endif()
if(TARGET p::curl)
add_definitions(-DHAS_CURL)
endif()
if(TARGET p::qrencode)
add_definitions(-DHAS_QRENCODE)
endif()
if(TARGET p::aruco)
add_definitions(-DHAS_ARUCO)
endif()


#Should be fixed in the future
#find_library(GSTREAMER_O     NAMES gobject-2.0)
#find_library(GSTREAMER       NAMES gstreamer-1.0)
#find_library(GSTREAMER_A     NAMES gstapp-1.0)
#find_library(GLIB            NAMES glib-2.0)
#set(GSTREAMER_ALL ${GSTREAMER} ${GSTREAMER_O} ${GSTREAMER_A} ${GLIB})
if(LINUX)
set(LINUXEXTRA_LIBRARIES rt boost_program_options boost_random)
endif()


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(sensor_manager   ${SRC}/main.cpp 
								${SRC}/session_manager.cpp 
								${SRC}/face_detector.cpp
								${SRC}/linemod.cpp 
								${SRC}/kinect_grabber.cpp 
								${SRC}/data_writer.cpp 
								${SRC}/mongoose.cpp 
								#${SRC}/x264decoder.cpp 
								${SRC}/x264encoder.cpp
								${SRC}/sse_handler.cpp 
								#${SRC}/xn16zdec.cpp
								${SRC}/hand_detector.cpp
								${SRC}/gstreamer_grabber.cpp
								${SRC}/opt.cpp
								#${SRC}/gstreamer_grabber2.cpp
								${SRC}/ide_handler.cpp
								${SRC}/audio_detector.cpp
								#${SRC}/end_dec.cpp
								${SRC}/k2g.cpp
								${SRC}/screen_grabber.cpp
								${SRC}/image_sender.cpp
								${SRC}/base64.cpp
								${SRC}/qr_creator.cpp
								${SRC}/upload.cpp
								${SRC}/calibrator.cpp
								${SRC}/aliver.cpp
								${SRC}/marker_viewer.cpp
								${SRC}/opt_parse.cpp
								)

target_link_libraries(sensor_manager 	
	p::opencv
	p::tinyxml2
	#p::boost
	p::json
	#p::cppnetlib
	#p::openssl
	p::zeromq
	p::x264
	p::ffmpeg
	#p::curl
	#p::aruco
	${LINUXEXTRA_LIBRARIES} 
	${BOOST_LIBRARIES} 
	${CPPNETLIB_LIBRARIES}
										
	p::gstreamer
	p::ffmpeg
	#p::freenect2
	p::glfw
	p::portaudio
	#p::qrencode
	p::cairo
	p::x11
	${PCL_LIBRARIES}
)
#display should be display2d


