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

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../external/yamlcpp ${CMAKE_CURRENT_BINARY_DIR}/yamlcppx)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../external/yamlcpp/include)


#include(${CMAKE_ROOT}/Modules/FindPkgConfig.cmake)
#find_package(PkgConfig REQUIRED)

#TODO1.58 is the first with ASIO
set(Boost_USE_MULTITHREADED ON)
set(CMAKEGO_BOOST random program_options filesystem system thread )
find_package(cmakego REQUIRED COMPONENTS boost json libusb lame yamlnew eigen cairo ffmpeg curl REQUIRED)

# these are optional
usepackage(opencv2 gstreamer freenect freenect2 tinyxml2 x264 ffmpeg  portaudio x264 tinyxml2 glfw zeromq)

find_package(Websocketpp REQUIRED)

message(NOTE Cppnetlib has to be at least 0.13 using boost::asio)
find_package(Cppnetlib REQUIRED)
add_library(p::cppnetlib INTERFACE IMPORTED)
set_property(TARGET p::cppnetlib PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
set_property(TARGET p::cppnetlib PROPERTY INTERFACE_LINK_LIBRARIES ${CPPNETLIB_LIBRARIES})

#ADD CURLPP
find_path ( CURLPP_INCLUDE_DIRS  NAMES   curlpp/cURLpp.hpp )
find_library ( CURLPP_LIBRARIES  NAMES    curlpp)
find_package_handle_standard_args ( curlpp DEFAULT_MSG CURLPP_INCLUDE_DIRS  CURLPP_LIBRARIES)
if(CURLPP_FOUND)
	add_library(p::curlpp INTERFACE IMPORTED)
	set_property(TARGET p::curlpp PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${CURLPP_INCLUDE_DIRS}")
	set_property(TARGET p::curlpp PROPERTY INTERFACE_LINK_LIBRARIES ${CURLPP_LIBRARIES} p::curl)
endif()		

# this function maps the presence of a library to a macro definition in the code
function(optlibrary NAME FLAG)
	if(TARGET ${NAME})
		add_definitions(-D${FLAG})
		set(EXTRA_LIBRARIES ${EXTRA_LIBRARIES} ${NAME})
	endif()
endfunction(optlibrary)			

optlibrary(p::curlpp HAS_CURL)
optlibrary(p::qrencode HAS_QRENCODE)
optlibrary(p::aruco HAS_ARUCO)
optlibrary(p::gstreamer HAS_GSTREAMER)
optlibrary(p::freenect2 HAS_FREENECT2)

if(LINUX)
	set(EXTRA_LIBRARIES ${EXTRA_LIBRARIES}  rt p::x11)
endif()


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(sensor_core STATIC 
								${SRC}/data_writer.cpp 
								${SRC}/gstreamer_grabber.cpp
								${SRC}/trigger.cpp
								${SRC}/termination.cpp
        ${SRC}/param_storage.cpp
								${SRC}/base64.cpp


	)

add_executable(sensor_manager   ${SRC}/main.cpp 
								${SRC}/session_manager.cpp 
								${SRC}/face_detector.cpp
								#${SRC}/linemod.cpp
								#${SRC}/kinect_grabber.cpp
								${SRC}/mongoose.cpp 
								#${SRC}/x264decoder.cpp 
								${SRC}/x264encoder.cpp
								${SRC}/sse_handler.cpp 
								#${SRC}/xn16zdec.cpp
								${SRC}/hand_detector.cpp
								${SRC}/opt.cpp
								#${SRC}/gstreamer_grabber2.cpp
								${SRC}/ide_handler.cpp
								${SRC}/audio_detector.cpp
								#${SRC}/end_dec.cpp
								${SRC}/k2g.cpp
        ${SRC}/webcam_publisher.cpp
								${SRC}/screen_shotter.cpp
								${SRC}/screen_grabber.cpp
								${SRC}/video_saver.cpp
								${SRC}/image_sender.cpp
								${SRC}/qr_creator.cpp
								${SRC}/upload.cpp
								${SRC}/oni_compressor.cpp
								${SRC}/calibrator.cpp
								${SRC}/aliver.cpp
								${SRC}/marker_viewer.cpp
								${SRC}/opt_parse.cpp
								)

message(STATUS ${CMAKEGO_LIBRARIES})
target_link_libraries(sensor_manager
        yaml-cppx
        p::cppnetlib 
        sensor_core
        ${CMAKEGO_LIBRARIES}
		${PCL_LIBRARIES}
		${EXTRA_LIBRARIES}
)

add_executable(sensor_managertest ${SRC}/sensor_managertest.cpp ${SRC}/video_saver.cpp ${SRC}/webcam_publisher.cpp)

target_link_libraries(sensor_managertest
        yaml-cppx
        p::cppnetlib 
        sensor_core
        ${CMAKEGO_LIBRARIES}
		${PCL_LIBRARIES}
		${EXTRA_LIBRARIES}
)

#display should be display2d


