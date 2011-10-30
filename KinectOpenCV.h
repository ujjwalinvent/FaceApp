/*
 * KinectOpenCV.h
 *
 *  Created on: 11 Dec 2010
 *      Author: Murilo Fernandes Martins
 *      Email: muhrix@gmail.com
 *
 */

#ifndef KINECTOPENCV_H_
#define KINECTOPENCV_H_

#include <stdexcept>

#include <cv.h>
#include <cxcore.h>
#include <cvaux.h>
#include <highgui.h>

#include <usb.h>
#include <libfreenect.h>

// Boost threads and binding stuff
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>


#define FREENECT_FRAME_W 640
#define FREENECT_FRAME_H 480
#define FREENECT_FRAME_PIX  (FREENECT_FRAME_H*FREENECT_FRAME_W)
#define FREENECT_VIDEO_RGB_SIZE (FREENECT_FRAME_PIX*3)
#define FREENECT_DEPTH_11BIT_SIZE (FREENECT_FRAME_PIX*sizeof(uint16_t))


class IMU {
public:
	IMU();
	IMU(const IMU&);
	virtual ~IMU();
	const IMU& operator=(const IMU&);
	double x;
	double y;
	double z;
};

class KinectOpenCV {
public:
	KinectOpenCV(int devID);
	virtual ~KinectOpenCV();

	void StartKinect(void);
	void StopKinect(void);

	int GetColourImage(IplImage *colour);
	int GetDepthImage(IplImage *depth);
	IMU GetIMUData(void);
	int GetTiltAngle(void);
	double GetDepthInMetres(int x, int y);

	void SetTiltAngle(int titl_ang);
	void SetLED(int led_mode);
	//----functions added by ujjwal
	bool GetDepthMap(uint16_t* depthMap);
	uint16_t GetRawDepthValue(int x, int y);

private:
	KinectOpenCV();
	// Handles for libfreenect
	freenect_context *fctx;
	freenect_device *fdev;

	// Callback functions for processing depth and color image buffers
	static void DepthImageCallback(freenect_device *dev, void *depth, uint32_t timestamp);
	static void ColourImageCallback(freenect_device *dev, void *rgb, uint32_t timestamp);

	void SetDepthImage(freenect_device *dev, void *depth, uint32_t timestamp);
	void SetColourImage(freenect_device *dev, void *rgb, uint32_t timestamp);

	bool RGB_running;
	bool depth_running;
	int devNum;
	int userDevNum;
	int newcdata;
	int newddata;
	int angle;
	int LED;
	IMU _imu;

	// Lookup table for depth image
	uint16_t t_gamma[2048];
	double t_gamma_metres[2048];

	IplImage* colourImg;
	uint16_t* depthBuffer;

	boost::shared_ptr<boost::thread> k_thread;
	boost::mutex mtx;
	boost::mutex colour_mtx;
	boost::mutex depth_mtx;
	volatile bool k_stop;

	void KinectThread(void);
};

#endif /* MYKINECTOPENCV_H_ */
