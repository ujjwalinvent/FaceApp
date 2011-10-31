/*
 * KinectOpenCV.cpp
 *
 *  Created on: 11 Dec 2010
 *      Author: Murilo Fernandes Martins
 *      Email: muhrix@gmail.com
 *
 */

#include "KinectOpenCV.h"
#include <libfreenect.h>

IMU::IMU():x(0.0), y(0.0), z(0.0) {

}

IMU::~IMU() {

}

IMU::IMU(const IMU& fd): x(fd.x), y(fd.y), z(fd.z) {

}

const IMU& IMU::operator=(const IMU& fd) {
	if( &fd == this ) {
		return fd;
	}
	this->x = fd.x;
	this->y = fd.y;
	this->z = fd.z;
	return *this;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

KinectOpenCV::KinectOpenCV() {

}

KinectOpenCV::KinectOpenCV(int devID):RGB_running(false), depth_running(false), devNum(0), userDevNum(devID),
		newcdata(0), newddata(0), angle(0), LED(LED_RED), k_stop(false) {
	//this->colourImg = NULL;
	this->colourImg = cvCreateImage( cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H), IPL_DEPTH_8U, 3 );
	cvZero(this->colourImg);
	this->depthBuffer = NULL;

	// Initialise the colour map lookup table (from glview.c)
	for( int i = 0; i < 2048; i++ ) {
		float v = i/2048.0;
		v = powf(v, 3) * 6;
		this->t_gamma[i] = v*6*256;
	}

	// The initialisation below comes from:
	// http://groups.google.com/group/openkinect/browse_thread/thread/31351846fd33c78/e98a94ac605b9f21?lnk=gst&q=stephane
	//for( int i = 0; i < 2048; i++ ) {
	//	this->t_gamma_metres[i] = 0.1236 * tanf((float)i/2842.5 + 1.1863);
	//}

	// The initialisation below comes from:
	// http://nicolas.burrus.name/index.php/Research/KinectCalibration
	for( int i = 0; i < 2048; i++ ) {
		this->t_gamma_metres[i] = 1.0 / ((double)i * -0.0030711016 + 3.3309495161);
	}

	if( freenect_init(&this->fctx, NULL) < 0 ) {
		throw std::runtime_error("Cannot initialise freenect library");
	}

	//freenect_set_log_level(this->fctx, FREENECT_LOG_DEBUG); // this outputs "Write Reg 0x0005 <= 0x00" and so on
	freenect_set_log_level(this->fctx, FREENECT_LOG_WARNING); // I saw no outputs on the console using this level (probably a good sign)
	//freenect_set_log_level(this->fctx, FREENECT_LOG_INFO); // this outputs info such as "Device 0x6f4b40 open during shutdown, closing..."

	if( (this->devNum = freenect_num_devices(this->fctx)) < 1 ) {
		throw std::runtime_error("No Kinect device found");
	}
}

KinectOpenCV::~KinectOpenCV() {
	{
		boost::mutex::scoped_lock lock(this->mtx);
		this->k_stop = true;
	}
	//this->cond.notify_one();
	this->k_thread->join();

	freenect_set_tilt_degs(this->fdev, 0);
	freenect_set_led(this->fdev, LED_RED);


	if( this->RGB_running ) {
		if( freenect_stop_video(this->fdev) != 0 ) {
			throw std::runtime_error("Cannot stop video callback");
		}
	}

	if( this->depth_running ) {
		if( freenect_stop_depth(this->fdev) != 0 ) {
			throw std::runtime_error("Cannot stop depth callback");
		}
	}

	if( freenect_shutdown(this->fctx) < 0 ) {
		throw std::runtime_error("Cannot cleanup freenect library");
	}

	if( this->colourImg != NULL ) {
		cvReleaseImage(&this->colourImg);
	}

	if( this->depthBuffer != NULL ) {
		delete[] this->depthBuffer;
	}
}

void KinectOpenCV::StartKinect(void) {
	if( freenect_open_device(this->fctx, &this->fdev, this->userDevNum) < 0 ) {
		throw std::runtime_error("Cannot open Kinect device");
	}

	freenect_set_user(this->fdev, this);

	if( (this->k_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&KinectOpenCV::KinectThread, this)))) == NULL ) {
		throw std::runtime_error("Cannot create Kinect thread");
	}
	freenect_set_led(this->fdev, LED_YELLOW);
}

void KinectOpenCV::StopKinect(void) {
	if( freenect_stop_video(this->fdev) != 0 ) {
		throw std::runtime_error("Cannot stop video callback");
	}
	this->RGB_running = false;

	if( freenect_stop_depth(this->fdev) != 0 ) {
		throw std::runtime_error("Cannot stop depth callback");
	}
	this->depth_running = false;

	freenect_set_led(this->fdev, LED_YELLOW);
}

void KinectOpenCV::KinectThread(void) {
	freenect_set_tilt_degs(this->fdev, this->angle);
	freenect_set_led(this->fdev,LED_GREEN);

	freenect_set_depth_callback(this->fdev, DepthImageCallback);
	freenect_set_video_callback(this->fdev, ColourImageCallback);

	freenect_set_video_mode(this->fdev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
	freenect_set_depth_mode(this->fdev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));

	if( freenect_start_video(this->fdev) != 0 ) {
		throw std::runtime_error("Cannot start video callback");
	}
	this->RGB_running = true;

	if( freenect_start_depth(this->fdev) != 0 ) {
		throw std::runtime_error("Cannot start depth callback");
	}
	this->depth_running = true;

	double dx,dy,dz;
	int tilt_degs;

	for(;;) {
		if( freenect_process_events(this->fctx) != 0 ) {
			throw std::runtime_error("Cannot process Kinect events");
		}
		freenect_raw_tilt_state* state;
		freenect_update_tilt_state(this->fdev);
		state = freenect_get_tilt_state(this->fdev);
		tilt_degs = freenect_get_tilt_degs(state);
		freenect_get_mks_accel(state, &dx, &dy, &dz);

		{
			boost::mutex::scoped_lock u_lock(this->mtx);
			this->angle = tilt_degs;
			this->_imu.x = dx;
			this->_imu.y = dy;
			this->_imu.z = dz;

			if( this->k_stop ) {
				break;
			}
		}
	}
	return;
}

void KinectOpenCV::DepthImageCallback(freenect_device *dev, void *_depth, uint32_t timestamp) {
	KinectOpenCV* device = static_cast<KinectOpenCV*>(freenect_get_user(dev));
	device->SetDepthImage(dev, _depth, timestamp);
}

void KinectOpenCV::ColourImageCallback(freenect_device *dev, void *_rgb, uint32_t timestamp) {
	KinectOpenCV* device = static_cast<KinectOpenCV*>(freenect_get_user(dev));
	device->SetColourImage(dev, _rgb, timestamp);
}

void KinectOpenCV::SetDepthImage(freenect_device *dev, void *_depth, uint32_t timestamp) {
	boost::mutex::scoped_lock u_lock(this->depth_mtx);
	uint16_t* depth = static_cast<uint16_t*>(_depth);
	if( this->depthBuffer != NULL ) {
		delete[] this->depthBuffer;
		this->depthBuffer = NULL;
	}
	this->depthBuffer = new uint16_t[FREENECT_DEPTH_11BIT_SIZE];

	memcpy(this->depthBuffer, depth, FREENECT_DEPTH_11BIT_SIZE);
	this->newddata = 1;
}

void KinectOpenCV::SetColourImage(freenect_device *dev, void *_rgb, uint32_t timestamp) {
	boost::mutex::scoped_lock u_lock(this->colour_mtx);
	uint8_t* rgb = static_cast<uint8_t*>(_rgb);

	if( this->colourImg == NULL ) {
		throw std::runtime_error("colourImg is NULL");
	}
	memcpy(this->colourImg->imageData, rgb, FREENECT_VIDEO_RGB_SIZE);
	this->newcdata = 1;
}

int KinectOpenCV::GetColourImage(IplImage *colour) {
	if( colour == NULL ) {
		throw std::runtime_error("KinectOpenCV::GetColourImage(): Argument is NULL");
	}

	if( !(colour->width == FREENECT_FRAME_W || colour->width == (FREENECT_FRAME_W/2)) ||
			!(colour->height == FREENECT_FRAME_H || colour->height == (FREENECT_FRAME_H/2) ) ||
			  colour->nChannels != 3 ) {
		throw std::runtime_error("KinectOpenCV::GetColourImage(): Argument has invalid attribute(s)");
	}
	boost::mutex::scoped_lock u_lock(this->colour_mtx);
	if( this->newcdata > 0 ) {
		if(colour->width == FREENECT_FRAME_W || colour->height == FREENECT_FRAME_H) {
			cvCvtColor(this->colourImg, colour, CV_RGB2BGR);
		}
		else {
			cvCvtColor(this->colourImg, this->colourImg, CV_RGB2BGR);
			cvResize(this->colourImg, colour, CV_INTER_LINEAR);
		}
		this->newcdata--;
		return 0;
	}
	return 1;
}

int KinectOpenCV::GetDepthImage(IplImage *depth) {
	if( depth == NULL ) {
		throw std::runtime_error("KinectOpenCV::GetDepthImage(): Argument is NULL");
	}

	if( depth->width != FREENECT_FRAME_W || depth->height != FREENECT_FRAME_H || depth->nChannels != 3 || depth->depth != IPL_DEPTH_8U ) {
		throw std::runtime_error("KinectOpenCV::GetDepthImage(): Argument has invalid attribute(s)");
	}
	boost::mutex::scoped_lock u_lock(this->depth_mtx);
	if( this->newddata > 0 ) {
		//for( int i = 0; i < FREENECT_FRAME_PIX; i++ ) {
		for( int x = 0; x < FREENECT_FRAME_W; x++ ) {
			for( int y = 0; y < FREENECT_FRAME_H; y++ ) {
				int pval = this->t_gamma[this->depthBuffer[FREENECT_FRAME_W*y + x]];
				int lb = pval & 0xff;
				switch(pval >> 8) {
				// REMEMBER THAT IPLIMAGE FORMAT IS BGR, NOT RGB
				case 0:
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 2] = 255;		// RED
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 1] = 255 - lb;	// GREEN
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 0] = 255 - lb;	// BLUE
					break;
				case 1:
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 2] = 255;		// RED
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 1] = lb;		// GREEN
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 0] = 0;			// BLUE
					break;
				case 2:
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 2] = 255 - lb;	// RED
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 1] = 255;		// GREEN
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 0] = 0;			// BLUE
					break;
				case 3:
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 2] = 0;			// RED
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 1] = 255;		// GREEN
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 0] = lb;		// BLUE
					break;
				case 4:
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 2] = 0;			// RED
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 1] = 255 - lb;	// GREEN
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 0] = 255;		// BLUE
					break;
				case 5:
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 2] = 0;			// RED
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 1] = 0;			// GREEN
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 0] = 255 - lb;	// BLUE
					break;
				default:
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 2] = 0;			// RED
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 1] = 0;			// GREEN
					((uchar*)(depth->imageData + depth->widthStep*y))[x*3 + 0] = 0;			// BLUE
					break;
				}
			}
		}
		this->newddata--;
		return 0;
	}
	return 1;
}

IMU KinectOpenCV::GetIMUData(void) {
	boost::mutex::scoped_lock u_lock(this->mtx);
	return this->_imu;
}

int KinectOpenCV::GetTiltAngle(void) {
	boost::mutex::scoped_lock u_lock(this->mtx);
	return this->angle;
}

double KinectOpenCV::GetDepthInMetres(int x, int y) {
	if( x < 0 || x >= FREENECT_FRAME_W || y < 0 || y >= FREENECT_FRAME_H ) {
		throw std::runtime_error("GetDepthInMetres(): Invalid values of arguments");
	}

	boost::mutex::scoped_lock u_lock(this->depth_mtx);
	if( this->depthBuffer != NULL ) {
		return this->t_gamma_metres[this->depthBuffer[FREENECT_FRAME_W*y + x]];
	}
	// there is no depth buffer at the moment... return error code
	return -1.0;
}

void KinectOpenCV::SetTiltAngle(int tilt_ang) {
	if( tilt_ang > 30 || tilt_ang < -30 ) {
		throw std::runtime_error("Tilt angle out of range");
	}
	freenect_set_tilt_degs(this->fdev, tilt_ang);
}

void KinectOpenCV::SetLED(int led_mode) {
	if( led_mode < 0 || led_mode > 6 ) {
		throw std::runtime_error("Invalid LED mode");
	}
	else if (led_mode == '1') {
		freenect_set_led(this->fdev, LED_GREEN);
	}
	else if (led_mode == '2') {
		freenect_set_led(this->fdev, LED_RED);
	}
	else if (led_mode == '3') {
		freenect_set_led(this->fdev, LED_YELLOW);
	}
	else if (led_mode == '4') {
		freenect_set_led(this->fdev, LED_BLINK_GREEN);
	}
	else if (led_mode == '5') {
		freenect_set_led(this->fdev, LED_BLINK_GREEN);
	}
	else if (led_mode == '6') {
		freenect_set_led(this->fdev, LED_BLINK_RED_YELLOW);
	}
	else if (led_mode == '0') {
		freenect_set_led(this->fdev, LED_OFF);
	}
}



//------------code Added by ujjwal--------------------------

bool KinectOpenCV::GetDepthMap(uint16_t* depthMap) {
	boost::mutex::scoped_lock u_lock(this->depth_mtx); 
	if( this->depthBuffer != NULL ) {
		memcpy(depthMap,this->depthBuffer,FREENECT_DEPTH_11BIT_SIZE);
		return true;
	}
	// there is no depth buffer at the moment... return error code
	return false;
}

uint16_t KinectOpenCV::GetRawDepthValue(int x, int y)
{
	if( x < 0 || x >= FREENECT_FRAME_W || y < 0 || y >= FREENECT_FRAME_H ) {
		throw std::runtime_error("GetDepthInMetres(): Invalid values of arguments");
	}

	boost::mutex::scoped_lock u_lock(this->depth_mtx);
	if( this->depthBuffer != NULL ) {
		return this->depthBuffer[FREENECT_FRAME_W*y + x];
	}
	// there is no depth buffer at the moment... return error code
	return 0;
}
