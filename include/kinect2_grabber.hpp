//Copyright 2014 Giacomo Dabisias
//
//Licensed under the Apache License, Version 2.0 (the "License");
//you may not use this file except in compliance with the License.
//You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
//Unless required by applicable law or agreed to in writing, software
//distributed under the License is distributed on an "AS IS" BASIS,
//WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//See the License for the specific language governing permissions and
//limitations under the License.
//This is preliminary software and/or hardware and APIs are preliminary and subject to change.
#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "GL/glew.h"
//#include <libfreenect2/opengl.h>
#include <signal.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/registration.h>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/rgb_packet_stream_parser.h>

#include <string>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <sys/time.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct microser
{
	microser(std::ostream & ons) : ons_(ons)
	{}

	std::ostream & ons_;
};

inline microser & operator << (microser & x, const uint32_t & y)
{
	x.ons_.write((const char*)&y,4);
	return x;
}

inline microser & operator << (microser & x, const float & y)
{
	x.ons_.write((const char*)&y,4);
	return x;
	
}

inline microser & operator << (microser & x, const uint8_t &y)
{
	x.ons_.write((const char*)&y,1);
	return x;
	
}
 
inline microser & operator << (microser & x, const double & y)
{
	x.ons_.write((const char*)&y,8);
	return x;
	
}

inline microser & operator << (microser & x, const uint64_t & y)
{
	x.ons_.write((const char*)&y,8);
	return x;
}

BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)
namespace boost {
namespace serialization {
 
    /** Serialization support for cv::Mat */
    template <class Archive>
    void save(Archive & ar, const ::cv::Mat & m, const unsigned int version)
    {
		size_t elem_size = m.elemSize();
		size_t elem_type = m.type();

		ar & m.cols;
		ar & m.rows;
		ar & elem_size;
		ar & elem_type;

		const size_t data_size = m.cols * m.rows * elem_size;
		ar & boost::serialization::make_array(m.ptr(), data_size);
    }
 
    /** Serialization support for cv::Mat */
    template <class Archive>
    void load(Archive & ar, ::cv::Mat& m, const unsigned int version)
    {
		int cols, rows;
		size_t elem_size, elem_type;

		ar & cols;
		ar & rows;
		ar & elem_size;
		ar & elem_type;

		m.create(rows, cols, elem_type);

		size_t data_size = m.cols * m.rows * elem_size;
		ar & boost::serialization::make_array(m.ptr(), data_size);
    }
 
}

}

namespace Kinect2Grabber {

template <typename PointT>
class Frame;

template <typename PointT>
class CvFrameRgb;

template< typename PointT>
class Kinect2Grabber
{
friend class Frame<PointT>;
friend class CvFrameRgb<PointT>;
public:

	Kinect2Grabber( std::string rgb_image_folder_path,  std::string depth_image_folder_path, const int image_number, const cv::Size & board_size, const double square_size): 
				    serialize_(false), init_rototranslation_(false)
	{
		//glfwInit();
		dev_ = freenect2_.openDefaultDevice();
		listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
		if(dev_ == 0){
			std::cout << "no device connected or failure opening the default one!" << std::endl;
			exit(1);
		}
		dev_->setColorFrameListener(listener_);
		dev_->setIrAndDepthFrameListener(listener_);
		dev_->start();
		std::cout<< "starting calibration" << std::endl;
		calibrateCamera(rgb_image_folder_path, depth_image_folder_path, image_number, board_size, square_size);
		std::cout << "finished calibration" <<std::endl;
		std::cout << "device initialized" << std::endl;
		std::cout << "device serial: " << dev_->getSerialNumber() << std::endl;
		std::cout << "device firmware: " << dev_->getFirmwareVersion() << std::endl;

		init();
	}

	Kinect2Grabber( const std::string rgb_calibration_file, const std::string depth_calibration_file, const std::string pose_calibration_file): 
					serialize_(false), init_rototranslation_(false)
	{

		//glfwInit();
		dev_ = freenect2_.openDefaultDevice();
		listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
		if(dev_ == 0){
			std::cout << "no device connected or failure opening the default one!" << std::endl;
			exit(1);
		}
		dev_->setColorFrameListener(listener_);
		dev_->setIrAndDepthFrameListener(listener_);
		dev_->start();

		std::cout << "device initialized" << std::endl;
		std::cout << "device serial: " << dev_->getSerialNumber() << std::endl;
		std::cout << "device firmware: " << dev_->getFirmwareVersion() << std::endl;

		loadCalibration(rgb_calibration_file, depth_calibration_file, pose_calibration_file );

		printCalibration();
		init();
	}

	Kinect2Grabber( ): 
					serialize_(false), init_rototranslation_(false)
	{

		//glfwInit();
		dev_ = freenect2_.openDefaultDevice();
		ir_camera_params_ = dev_->getIrCameraParams();
  		rgb_camera_params_ = dev_->getColorCameraParams();
		registration_ = new libfreenect2::Registration(&ir_camera_params_, &rgb_camera_params_);

		listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
		if(dev_ == 0){
			std::cout << "no device connected or failure opening the default one!" << std::endl;
			exit(1);
		}
		dev_->setColorFrameListener(listener_);
		dev_->setIrAndDepthFrameListener(listener_);
		dev_->start();

		std::cout << "device initialized" << std::endl;
		std::cout << "device serial: " << dev_->getSerialNumber() << std::endl;
		std::cout << "device firmware: " << dev_->getFirmwareVersion() << std::endl;
		initAutoRegistered();
	}


	~Kinect2Grabber(){
		this->shutDown();
		if(file_streamer_->is_open())
			file_streamer_->close();
	}

	void
	printCalibration() const {

		std::cout << std::endl;
		std::cout <<"Single Calibration" <<std::endl;
		std::cout << std::endl;
		std::cout << "rgb :" <<std::endl;
		std::cout << std::endl;
		std::cout << "Camera Matrix:" <<std::endl ;
		std::cout << calibrgb_camera_matrix_ << std::endl;
		std::cout << std::endl;
		std::cout << "Distortion:" <<std::endl ;
		std::cout << rgb_distortion_ << std::endl;
		std::cout << std::endl;

		std::cout << "Depth :" <<std::endl;
		std::cout << std::endl;
		std::cout << "Camera Matrix:" <<std::endl ;
		std::cout << std::endl;
		std::cout << calibdepth_camera_matrix_ << std::endl;
		std::cout << std::endl;
		std::cout << "Distortion:" <<std::endl ;
		std::cout << std::endl;
		std::cout<< depth_distortion_ << std::endl;
		std::cout << std::endl;

		std::cout << std::endl;
		std::cout <<"Stereo Calibration :" <<std::endl;
		std::cout << std::endl;
		std::cout << "rotation:" << std::endl; 
		std::cout << std::endl;
		std::cout << rotation_ << std::endl;
		std::cout << std::endl;
		std::cout << "translation" << std::endl;
		std::cout << std::endl;
		std::cout << translation_ << std::endl;
		std::cout << std::endl;
		std::cout << "essential:"  << essential_ << std::endl;
		std::cout << std::endl;
		std::cout << "fundamental" <<  fundamental_ << std::endl;
		std::cout << std::endl;
	}

	void
	setDistance(const float dist){
		distance_ = dist;
	}


	libfreenect2::FrameMap *
	getRawFrames() {
		//using namespace std::chrono;
		//static high_resolution_clock::time_point last;

		// auto tnow = high_resolution_clock::now();
		listener_->waitForNewFrame(frames_);
		// auto tpost = high_resolution_clock::now();
		 //std::cout << "delta " << duration_cast<duration<double>>(tpost-last).count()*1000 << " " << duration_cast<duration<double>>(tpost-tnow).count()*1000 << std::endl;
		 //last = tpost;
	return &frames_;
	}

	void
	shutDown(){
		dev_->stop();
		dev_->close();
	}

	cv::Mat
	getCameraMatrixColor() const {
		return rgb_camera_matrix_;
	}

	cv::Mat
	getCalibCameraMatrixColor() const {
		return calibrgb_camera_matrix_;
	}

	cv::Mat
	getCameraMatrixDepth() const {
		return depth_camera_matrix_;
	}

	cv::Mat
	getCalibCameraMatrixDepth() const {
		return calibdepth_camera_matrix_;
	}

	cv::Mat
	getRgbDistortion() const {
		return rgb_distortion_;
	}

	cv::Mat
	getDepthDistortion() const {
		return depth_distortion_;
	}

	void
	freeFrames(){
		listener_->release(frames_);
	}

	void
	enableSerialization(){
		serialize_ = true;
	}

	void
	disableSerialization(){
		serialize_ = false;
	}


private:

	libfreenect2::Frame *
	getRgbFrame() {
		listener_->waitForNewFrame(frames_);
		return frames_[libfreenect2::Frame::Color];
	}

	libfreenect2::Frame *
	getIrFrame() {
		listener_->waitForNewFrame(frames_);
		return frames_[libfreenect2::Frame::Ir];
	} 

	libfreenect2::Frame *
	getDepthFrame() {
		listener_->waitForNewFrame(frames_);
		return frames_[libfreenect2::Frame::Depth];
	}

	void initSizeAndData(const int size_x=512 , const int size_y=424)
	{
	
		const float sx_depth =  ((float)size_x / (float)calibsize_depth_.width);
		const float sy_depth =  ((float)size_y / (float)calibsize_depth_.height);
		const float sx_rgb = ((float)size_x / (float)calibsize_rgb_.width);
		const float sy_rgb =  ((float)size_y / (float)calibsize_rgb_.height);

		depth_camera_matrix_ = calibdepth_camera_matrix_;
		depth_camera_matrix_.at<double>(0,0) *= sx_depth;
		depth_camera_matrix_.at<double>(1,1) *= sy_depth;
		depth_camera_matrix_.at<double>(0,2) *= sx_depth;
		depth_camera_matrix_.at<double>(1,2) *= sy_depth;

		//need to rescale since rgb image is resized
		rgb_camera_matrix_ = calibrgb_camera_matrix_;
		rgb_camera_matrix_.at<double>(0,0) *= sx_rgb;
		rgb_camera_matrix_.at<double>(1,1) *= sy_rgb;
		rgb_camera_matrix_.at<double>(0,2) *= sx_rgb;
		rgb_camera_matrix_.at<double>(1,2) *= sy_rgb;

		ir_fx_ = depth_camera_matrix_.at<double>(0,0);
		ir_fy_ = depth_camera_matrix_.at<double>(1,1);
		ir_cx_ = depth_camera_matrix_.at<double>(0,2);
		ir_cy_ = depth_camera_matrix_.at<double>(1,2);

		rgb_fx_ = rgb_camera_matrix_.at<double>(0,0);
		rgb_fy_ = rgb_camera_matrix_.at<double>(1,1);
		rgb_cx_ = rgb_camera_matrix_.at<double>(0,2);
		rgb_cy_ = rgb_camera_matrix_.at<double>(1,2);

		cv::initUndistortRectifyMap(depth_camera_matrix_, depth_distortion_, cv::Mat(), depth_camera_matrix_, cv::Size(size_x, size_y), CV_32FC1, map_x_depth_, map_y_depth_);
		//cv::initUndistortRectifyMap(rgb_camera_matrix_, rgb_distortion_, cv::Mat(), rgb_camera_matrix_,  cv::Size(size_x, size_y), CV_32FC1, map_x_rgb_, map_y_rgb_);
		//cv::initUndistortRectifyMap(calibdepth_camera_matrix_, depth_distortion_, cv::Mat(), depth_camera_matrix_, cv::Size(1024, 848), CV_32FC1, map_x_depth_, map_y_depth_);

	}

	void
	init(const int size_x=512 , const int size_y=424){
		initSizeAndData(size_x,size_y);
		unsigned threads = omp_get_max_threads();
		partial_clouds_.resize(threads);
		for(int i = 0; i < threads; ++i)
			partial_clouds_[i].reserve((size_x * size_y) / threads + 1 );
		distance_ = 10000;
	}

	void
	initAutoRegistered(const int size_x=512 , const int size_y=424){
		unsigned threads = omp_get_max_threads();
		partial_clouds_.resize(threads);
		for(int i = 0; i < threads; ++i)
			partial_clouds_[i].reserve((size_x * size_y) / threads + 1 );
		distance_ = 10000;
	}


 
	void
	loadCalibration(const std::string rgb_calibration_file, const std::string depth_calibration_file, const std::string pose_calibration_file ){

		cv::FileStorage fs;
		fs.open(rgb_calibration_file, cv::FileStorage::READ);
		int x,y;
		if(fs.isOpened())
	    {	
	    	fs["image_width"] >> x;
	    	fs["image_height"] >> y;
	    	calibsize_rgb_ = cv::Size(x,y);
			fs["camera_matrix"] >> calibrgb_camera_matrix_;
			fs["distortion_coefficients"] >> rgb_distortion_;
			fs.release();
	    }else{
	    	std::cout << "could not find rgb calibration file " << rgb_calibration_file <<std::endl;
	    	exit(-1);
	    }

	    fs.open(depth_calibration_file, cv::FileStorage::READ);

	    if(fs.isOpened())
	    {	    
	    	fs["image_width"] >> x;
	    	fs["image_height"] >> y;
	    	calibsize_depth_ = cv::Size(x,y);
			fs["camera_matrix"] >> calibdepth_camera_matrix_;
			fs["distortion_coefficients"] >> depth_distortion_;
			fs.release();
	    }else{
	    	std::cout << "could not find ir calibration file " << depth_calibration_file <<std::endl;
	    	exit(-1);
	    }

	    fs.open(pose_calibration_file, cv::FileStorage::READ);

	    if(fs.isOpened())
	    {	      
			fs["rotation matrix"] >> rotation_;
			fs["translation matrix"] >> translation_;
			fs["fundamental matrix"] >> fundamental_;
			fs["essential matrix"] >> essential_;
			fs.release();
	    }else{
	    	std::cout << "could not find pose calibration file " << pose_calibration_file <<std::endl;
	    	exit(-1);
	    }
	}

	void 
	calcBoardCornerPositions(const cv::Size & board_size, const float square_size_, std::vector<cv::Point3f> & corners)
	{
	    corners.clear();
	        for( int i = 0; i < board_size.height; ++i )
	            for( int j = 0; j < board_size.width; ++j )
	                corners.push_back(cv::Point3f(float( j*square_size_ ), float( i*square_size_ ), 0));
	}

	void 
	saveCameraParams( const std::string & filename, const cv::Size & image_size,
					  const cv::Size & board_size, const float square_size_,
					  const float aspect_ratio, const int flags,
                      const cv::Mat & camera_matrix, const cv::Mat & distortion,
                      const double total_error ) const {

		cv::FileStorage fs( filename, cv::FileStorage::WRITE );

		time_t t;
		time( &t );
		struct tm *t2 = localtime( &t );
		char buf[1024];
		strftime( buf, sizeof(buf)-1, "%c", t2 );

		fs << "calibration_time" << buf;
		fs << "image_width" << image_size.width;
		fs << "image_height" << image_size.height;
		fs << "board_width" << board_size.width;
		fs << "board_height" << board_size.height;
		fs << "square_size_" << square_size_;

		if( flags & cv::CALIB_FIX_ASPECT_RATIO )
		fs << "aspectRatio" << aspect_ratio;

		if( flags != 0 )
		{
		sprintf( buf, "flags: %s%s%s%s",
		flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
		flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
		flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
		flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
		}

		fs << "flags" << flags;
		fs << "camera_matrix" << camera_matrix;
		fs << "distortion_coefficients" << distortion;
		fs << "avg_reprojection_error" << total_error;
 	}

 	void 
	savePoseParams( const std::string & filename,
                    const cv::Mat & rotation, const cv::Mat & translation,
                    const cv::Mat & essential, const cv::Mat & fundamental,
                    const double total_error ) const
	{

		 cv::FileStorage fs( filename, cv::FileStorage::WRITE );

		 time_t t;
		 time( &t );
		 struct tm *t2 = localtime( &t );
		 char buf[1024];
		 strftime( buf, sizeof(buf)-1, "%c", t2 );

		 fs << "calibration_time" << buf;
		 fs << "rotation matrix" << rotation;
		 fs << "translation matrix" << translation;
		 fs << "essential matrix" << essential;
		 fs << "fundamental matrix" << fundamental;
		 fs << "avg_reprojection_error" << total_error;
 	}

	void
	calibrateCamera( std::string & rgb_image_folder_path,  std::string & depth_image_folder_path, int image_number,const cv::Size & board_size, const double square_size){

		const cv::TermCriteria term_criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON);
		std::vector<std::vector<cv::Point2f>> rgbImagePoints;
  		std::vector<std::vector<cv::Point2f>> irImagePoints;
  		std::vector<cv::Mat> rvecs, tvecs;


  		if(rgb_image_folder_path.back() != '/')
  			rgb_image_folder_path += std::string("/");

  		if(depth_image_folder_path.back() != '/')
  			depth_image_folder_path += std::string("/");

		int count = 0;
		for(int i = 0; i < image_number; ++i, ++count){

			std::string rgb_name = rgb_image_folder_path + std::string("rgb_image_") + std::to_string(count) + std::string(".jpg");
		    std::string ir_name = depth_image_folder_path + std::string("ir_image_") + std::to_string(count) + std::string(".jpg");
		 
		    cv::Mat rgb_gray = cv::imread(rgb_name, 0);
		    cv::Mat ir_gray = cv::imread(ir_name, 0);

		    calibsize_depth_ = ir_gray.size();
		    calibsize_rgb_ = rgb_gray.size();
		    std::vector<cv::Point2f > camera1ImagePoints;
		    bool found1 = cv::findChessboardCorners(rgb_gray, board_size, camera1ImagePoints, cv::CALIB_CB_FAST_CHECK);

		    std::vector<cv::Point2f> camera2ImagePoints;
		    bool found2 = cv::findChessboardCorners(ir_gray, board_size, camera2ImagePoints, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

		    if(found1){
		    	cv::cornerSubPix(rgb_gray, camera1ImagePoints, cv::Size(11, 11), cv::Size(-1, -1), term_criteria);
		    	rgbImagePoints.push_back(camera1ImagePoints);
		    }

		    if(found2){
			    cv::cornerSubPix(ir_gray, camera2ImagePoints, cv::Size(11, 11), cv::Size(-1, -1), term_criteria);
			    irImagePoints.push_back(camera2ImagePoints);
			}
		}

		std::vector<std::vector<cv::Point3f>> pointsBoard(1);
		calcBoardCornerPositions(board_size, square_size, pointsBoard[0]);
		pointsBoard.resize(image_number,pointsBoard[0]);
		double error_1 = cv::calibrateCamera(pointsBoard, rgbImagePoints, calibsize_rgb_, calibrgb_camera_matrix_, rgb_distortion_,  rvecs,  tvecs);
		saveCameraParams("rgb_calibration.yaml",calibsize_rgb_, board_size, square_size, 0, 0, calibrgb_camera_matrix_, rgb_distortion_, error_1);

		double error_2 = cv::calibrateCamera(pointsBoard, irImagePoints, calibsize_depth_, calibdepth_camera_matrix_, depth_distortion_,  rvecs,  tvecs );
		saveCameraParams("depth_calibration.yaml",calibsize_depth_, board_size, square_size, 0, 0, calibdepth_camera_matrix_, depth_distortion_, error_2 );

		double rms = cv::stereoCalibrate(pointsBoard, rgbImagePoints, irImagePoints,
		                calibrgb_camera_matrix_, rgb_distortion_,
		                calibdepth_camera_matrix_, depth_distortion_,
		                calibsize_rgb_, rotation_, translation_, essential_, fundamental_,
		                term_criteria,
		                cv::CALIB_FIX_INTRINSIC
		                );

		printCalibration();

		std::cout << std::endl;
		std::cout << "rgb error:" << error_1 <<std::endl;
		std::cout << "depth error:" << error_2 <<std::endl;
		std::cout << "stereo error " << rms << std::endl;

		savePoseParams("pose_calibration.yaml", rotation_, translation_, essential_, fundamental_, rms);
	}

	void
	initRotoTranslation()
	{
		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > erotation((double*)(rotation_.data));
		Eigen::Matrix3d rotation = erotation;
		Eigen::Map<Eigen::Vector3d> etranslation ((double*)(translation_.data));
		Eigen::Vector3d translation = etranslation;
		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > edepth_matrix((double*)(depth_camera_matrix_.data));
		Eigen::Matrix3d depth_matrix = edepth_matrix;
		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > ergb_matrix((double*)(rgb_camera_matrix_.data));
		Eigen::Matrix3d rgb_matrix = ergb_matrix;

		Eigen::Matrix4d rototranslation = Eigen::Matrix4d::Zero();
		Eigen::Matrix4d odepth_matrix = Eigen::Matrix4d::Zero();
		Eigen::Matrix4d orgb_matrix = Eigen::Matrix4d::Zero(); 

		rototranslation.block<3,3>(0,0) = rotation;
		rototranslation.block<3,1>(0,3) = translation;
		rototranslation(3,3) = 1;

		odepth_matrix.block<3,3>(0,0) = depth_matrix;
		odepth_matrix(3,3) = 1;

		orgb_matrix.block<3,3>(0,0) = rgb_matrix;
		orgb_matrix(3,3) = 1;

		world2rgb_ = orgb_matrix * rototranslation;
		depth2world_ = odepth_matrix.inverse();
	}

	


	void
	serializeFrames(const cv::Mat & depth, const cv::Mat & color)
	{	
		struct timeval tv;
    	gettimeofday(&tv,NULL);
    	uint64_t now = tv.tv_sec*(uint64_t)1000000 + tv.tv_usec;
		if(file_streamer_ == 0){
			file_streamer_ = new std::ofstream();
			file_streamer_->open ("stream", std::ios::binary);
			oa_ = new boost::archive::binary_oarchive(*file_streamer_);
		}

		(*oa_) << now << depth << color;
	}


	void 
	remapDepth(const cv::Mat & depth, cv::Mat & scaled, const cv::Size & size_registered, const int type) const
	{
		scaled.create(size_registered, type);
		if(type == CV_16U)
		{
			#pragma omp parallel for
			for(size_t r = 0; r < (size_t)size_registered.height; ++r)
			{
				uint16_t *itO = scaled.ptr<uint16_t>(r);
				const float *itX = map_x_depth_.ptr<float>(r);
				const float *itY = map_y_depth_.ptr<float>(r);
				for(size_t c = 0; c < (size_t)size_registered.width; ++c, ++itO, ++itX, ++itY)
				{
					*itO = interpolate16(depth, *itX, *itY);
				}
			}
		}
		else
		{
			#pragma omp parallel for
			for(size_t r = 0; r < (size_t)size_registered.height; ++r)
			{
				float *itO = scaled.ptr<float>(r);
				const float *itX = map_x_depth_.ptr<float>(r);
				const float *itY = map_y_depth_.ptr<float>(r);
				for(size_t c = 0; c < (size_t)size_registered.width; ++c, ++itO, ++itX, ++itY)
				{
					*itO = interpolatef32(depth, *itX, *itY);
				}
			}			
		}
	}

	inline float 
	interpolatef32(const cv::Mat & in, const float x, const float y) const
	{
		const int xL = (int)floor(x);
		const int xH = (int)ceil(x);
		const int yL = (int)floor(y);
		const int yH = (int)ceil(y);

		if(xL < 0 || yL < 0 || xH >= in.cols || yH >= in.rows)
		{
			return 0;
		}

		const float pLT = in.at<float>(yL, xL);
		const float pRT = in.at<float>(yL, xH);
		const float pLB = in.at<float>(yH, xL);
		const float pRB = in.at<float>(yH, xH);
		int vLT = pLT > 0;
		int vRT = pRT > 0;
		int vLB = pLB > 0;
		int vRB = pRB > 0;
		int count = vLT + vRT + vLB + vRB;

		if(count < 3)
		{
			return 0;
		}

		const float avg = (pLT + pRT + pLB + pRB) / count;
		const float thres = 0.01 * avg;
		vLT = abs(pLT - avg) < thres;
		vRT = abs(pRT - avg) < thres;
		vLB = abs(pLB - avg) < thres;
		vRB = abs(pRB - avg) < thres;
		count = vLT + vRT + vLB + vRB;

		if(count < 3)
		{
			return 0;
		}

		double distXL = x - xL;
		double distXH = 1.0 - distXL;
		double distYL = y - yL;
		double distYH = 1.0 - distYL;
		distXL *= distXL;
		distXH *= distXH;
		distYL *= distYL;
		distYH *= distYH;
		const double tmp = sqrt(2.0);
		const double fLT = vLT ? tmp - sqrt(distXL + distYL) : 0;
		const double fRT = vRT ? tmp - sqrt(distXH + distYL) : 0;
		const double fLB = vLB ? tmp - sqrt(distXL + distYH) : 0;
		const double fRB = vRB ? tmp - sqrt(distXH + distYH) : 0;
		const double sum = fLT + fRT + fLB + fRB;

		return ((pLT * fLT +  pRT * fRT + pLB * fLB + pRB * fRB) / sum) + 0.5;
	}


	inline uint16_t 
	interpolate16(const cv::Mat & in, const float x, const float y) const
	{
		const int xL = (int)floor(x);
		const int xH = (int)ceil(x);
		const int yL = (int)floor(y);
		const int yH = (int)ceil(y);

		if(xL < 0 || yL < 0 || xH >= in.cols || yH >= in.rows)
		{
			return 0;
		}

		const uint16_t pLT = in.at<uint16_t>(yL, xL);
		const uint16_t pRT = in.at<uint16_t>(yL, xH);
		const uint16_t pLB = in.at<uint16_t>(yH, xL);
		const uint16_t pRB = in.at<uint16_t>(yH, xH);
		int vLT = pLT > 0;
		int vRT = pRT > 0;
		int vLB = pLB > 0;
		int vRB = pRB > 0;
		int count = vLT + vRT + vLB + vRB;

		if(count < 3)
		{
			return 0;
		}

		const uint16_t avg = (pLT + pRT + pLB + pRB) / count;
		const uint16_t thres = 0.01 * avg;
		vLT = abs(pLT - avg) < thres;
		vRT = abs(pRT - avg) < thres;
		vLB = abs(pLB - avg) < thres;
		vRB = abs(pRB - avg) < thres;
		count = vLT + vRT + vLB + vRB;

		if(count < 3)
		{
			return 0;
		}

		double distXL = x - xL;
		double distXH = 1.0 - distXL;
		double distYL = y - yL;
		double distYH = 1.0 - distYL;
		distXL *= distXL;
		distXH *= distXH;
		distYL *= distYL;
		distYH *= distYH;
		const double tmp = sqrt(2.0);
		const double fLT = vLT ? tmp - sqrt(distXL + distYL) : 0;
		const double fRT = vRT ? tmp - sqrt(distXH + distYL) : 0;
		const double fLB = vLB ? tmp - sqrt(distXL + distYH) : 0;
		const double fRB = vRB ? tmp - sqrt(distXH + distYH) : 0;
		const double sum = fLT + fRT + fLB + fRB;

		return ((pLT * fLT +  pRT * fRT + pLB * fLB + pRB * fRB) / sum) + 0.5;
	}

	libfreenect2::Freenect2 freenect2_;
	libfreenect2::Registration * registration_ = 0;
	libfreenect2::Freenect2Device * dev_ = 0;
	libfreenect2::SyncMultiFrameListener * listener_ = 0;
	libfreenect2::Frame * depth_ = 0;
	libfreenect2::Frame *rgb_ = 0;
	libfreenect2::FrameMap frames_;
	cv::Mat scaled_, registered_, rgb_scaled_, map_x_depth_, map_y_depth_,  map_x_rgb_, map_y_rgb_;
	cv::Mat rotation_, translation_, essential_, fundamental_;
	cv::Size size_registered_, calibsize_depth_, calibsize_rgb_;
	cv::Mat lookup_y_, lookup_x_, tmp_depth_, tmp_rgb_, depth_final_, rgb_final_;
  	cv::Mat calibrgb_camera_matrix_, calibdepth_camera_matrix_, rgb_camera_matrix_, depth_camera_matrix_, depth_distortion_, rgb_distortion_;
  	double rgb_fx_, rgb_fy_, rgb_cx_, rgb_cy_;
  	double ir_fx_, ir_fy_, ir_cx_, ir_cy_;
  	bool serialize_, init_rototranslation_;
  	float distance_;
  	std::ofstream * file_streamer_ = 0;
  	std::vector<std::vector<PointT>> partial_clouds_;
  	boost::archive::binary_oarchive * oa_ = 0;
  	Eigen::Matrix4d world2rgb_;
  	Eigen::Matrix4d depth2world_ ;
  	libfreenect2::Freenect2Device::IrCameraParams ir_camera_params_;
    libfreenect2::Freenect2Device::ColorCameraParams rgb_camera_params_;
  	

};

template <typename PointT>
class Frame{
public:

	libfreenect2::Frame * data_;

	Frame(Kinect2Grabber<PointT> & k): grabber_(k){
		data_ = grabber_.getRgbFrame();
	}

	~Frame(){
		grabber_.freeFrames();
	}

private:
	Kinect2Grabber<PointT> & grabber_; 
};

template <typename PointT>
class CvFrame{
public:

	cv::Mat data_;

	CvFrame(Kinect2Grabber<PointT> & k): grabber_(k){
		freenect_data_ = grabber_.getRgbFrame();
		data_ = cv::Mat(freenect_data_->height, freenect_data_->width, CV_8UC3, freenect_data_->data);
	}

	~CvFrame(){
		grabber_.freeFrames();
	}

private:
	Kinect2Grabber<PointT> & grabber_; 
	libfreenect2::Frame * freenect_data_;
};

template <typename PointT>
class CvFrameRgb{
public:

	cv::Mat data_;

	CvFrameRgb(Kinect2Grabber<PointT> & k): grabber_(k){
		freenect_data_ = grabber_.getRgbFrame();
		data_ = cv::Mat(freenect_data_->height, freenect_data_->width, CV_8UC3, freenect_data_->data);
	}

	~CvFrameRgb(){
		grabber_.freeFrames();
	}

private:
	Kinect2Grabber<PointT> & grabber_; 
	libfreenect2::Frame * freenect_data_;
};

template <typename PointT>
class CvFrameDepth{
public:

	cv::Mat data_, data2_;

	CvFrameDepth(Kinect2Grabber<PointT> & k): grabber_(k){
		std::cout << "getting freenect data depth" << std::endl;
		freenect_data_ = grabber_.getDepthFrame();
		std::cout << "copy freenect data depth" << std::endl;
		data2_ = cv::Mat(freenect_data_->height, freenect_data_->width, CV_32FC1, freenect_data_->data);
		data2_.convertTo(data_, CV_16U);
	}

	~CvFrameDepth(){
		grabber_.freeFrames();
	}

private:
	Kinect2Grabber<PointT> & grabber_; 
	libfreenect2::Frame * freenect_data_;
};


}

