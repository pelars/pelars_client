cmake_minimum_required(VERSION 2.8)
Project (sensor_manager CXX)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/modules")
set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -Wall")

set(CMAKE_BUILD_TYPE   Release) 

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
find_package(Boost COMPONENTS random program_options asio REQUIRED)
find_package(Websocketpp REQUIRED)
find_package(Gstreamer-1.0 REQUIRED)
find_package(Freenect REQUIRED)
find_package(FFmpeg REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(JsonCpp REQUIRED)
find_package(x264 REQUIRED)
find_package(Cppnetlib REQUIRED)
find_package(CURL REQUIRED)
add_definitions(-DHAS_CURL)
find_package(Aruco REQUIRED)
add_definitions(-DHAS_ARUCO)
find_package(PkgConfig REQUIRED)
find_package(FFmpeg REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(QREncode REQUIRED)
add_definitions(-DHAS_QRENCODE)
find_package(PortAudio REQUIRED)
find_package(Cairo REQUIRED)
find_package(Freenect2 REQUIRED)
add_definitions(-DHAS_FREENECT2)

set(Boost_USE_MULTITHREADED ON) 

set(X264 ${X264_LIBRARIES})
set(TINYXML2 ${TINYXML2_LIBRARY})

#Should be fixed in the future
find_library(GSTREAMER_O     NAMES gobject-2.0)
find_library(GSTREAMER       NAMES gstreamer-1.0)
find_library(GSTREAMER_A     NAMES gstapp-1.0)
find_library(GLIB            NAMES glib-2.0)
set(GSTREAMER_ALL ${GSTREAMER} ${GSTREAMER_O} ${GSTREAMER_A} ${GLIB})
set(LINUXEXTRA rt boost_program_options boost_random)

include_directories(${JSONCPP_INCLUDE_DIR})
include_directories(${FFMPEG_INCLUDE_DIRS})
include_directories(${GSTREAMER_INCLUDE_DIRS})
include_directories(${FREENECT_INCLUDE_DIRS})
include_directories(${FREENECT2_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${INCLUDE})
include_directories(${GLFW_INCLUDE_DIRS})
include_directories(${Boost_LIBRARY_DIRS})
include_directories(${CAIRO_INCLUDE_DIRS})

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

target_link_libraries(sensor_manager 	${OpenCV_LIBS} 
										${TINYXML2} 
										${BOOST_LIBRARIES} 
										${JSONCPP_LIBRARY}
										${CPPNETLIB_LIBRARIES}
										${OPENSSL_LIBRARIES} 
										${ZMQ_LIBRARY} 
										${X264} 
										${SWSCALE} 
										${AVFORMAT}
										${AVUTIL} 
										${AVCODEC} 
										${FFMPEG_LIBRARIES} 
										${AVDEVICE}
										${CURL_LIBRARIES}
										${ARUCO_LIBRARIES}
										${LINUXEXTRA} 
										${GSTREAMER_ALL}
										${FFMPEG_LIBRARIES}
										${FREENECT2_LIBRARY}
										${GLFW_LIBRARIES}
										${PCL_LIBRARIES}
										${PORTAUDIO_LIBRARIES}
										${QRENCODE_LIBRARY}
										${CAIRO_LIBRARIES}
										X11
										)

#add_executable(freenect-camshare ${SRC}/camshare.cpp)
#target_link_libraries(freenect-camshare ${FREENECT_LIBRARIES} rt) 


