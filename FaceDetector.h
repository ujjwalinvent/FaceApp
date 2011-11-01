#ifndef FACEDETECTOR_H_
#define FACEDETECTOR_H_
#include <cv.h>
#include <iostream>
#include <cxcore.h>
#include <cvaux.h>
#include <highgui.h>

using namespace cv;
class FaceDetector
{
public:
	CvRect faceRect;
	CvRect mouthRect;
	CvRect noseRect;
	CvRect leftEyeRect;
	CvRect rightEyeRect;
	String cascadeClassifier[5];
	CascadeClassifier faceCascade;
	CascadeClassifier leftEyeCascade;
	CascadeClassifier rightEyeCascade;
	CascadeClassifier noseCascade;
	CascadeClassifier mouthCascade;
	FaceDetector(void);
	~FaceDetector(void);
	void loadClassifiersFromFile();
	void readConfigFile();
	void init();
	void showImage(IplImage* rgb);
	bool detectFaceInImage(IplImage* rgb);
	bool detectLeftEyeInFace(Mat faceROI);
	bool detectRightEyeInFace(Mat faceROI);
	bool detectNoseInFace(Mat faceROI);
	bool detectMouthInFace(Mat faceROI);
	CvRect getFaceRect();
	CvRect getLeftEyeRect();
	CvRect getRightEyeRect();
	CvRect getNoseRect();
	CvRect getMouthRect();
	String toString();
	ostream operatorOut();
};

#endif