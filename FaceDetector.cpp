#include "FaceDetector.h"


FaceDetector::FaceDetector(void)
{
		cascadeClassifier[0]="C:\\OpenCV2.2\\data\\haarcascades\\haarcascade_frontalface_alt.xml";
		cascadeClassifier[1]="C:\\OpenCV2.2\\data\\haarcascades\\haarcascade_mcs_lefteye.xml";
		cascadeClassifier[2]="C:\\OpenCV2.2\\data\\haarcascades\\haarcascade_mcs_righteye.xml";
		cascadeClassifier[3]="C:\\OpenCV2.2\\data\\haarcascades\\haarcascade_mcs_nose.xml";
		cascadeClassifier[4]="C:\\OpenCV2.2\\data\\haarcascades\\haarcascade_mcs_mouth.xml";
}


FaceDetector::~FaceDetector(void)
{

}

bool FaceDetector::detectFaceInImage(IplImage *rgb)
{
	std::vector<Rect> faces;
	Mat colour_gray;
	Mat colour(rgb);
	cvtColor( colour, colour_gray, CV_BGR2GRAY );
	equalizeHist( colour_gray, colour_gray );
	if( !faceCascade.load(cascadeClassifier[0]) )
	{ 
		std::cout<<"--(!)Error loading\n";
		exit(-1);
	}
	faceCascade.detectMultiScale(colour_gray,faces,1.1,2,0);
	if(faces.size()==0) return false;
	for(int i=0;i<faces.size();i++)
	{
		CvPoint p1,p2;
		p1.x=faces[i].x;
		p1.y=faces[i].y;
		p2.x=faces[i].x+faces[i].width;
		p2.y=faces[1].y+faces[i].height;
		faceRect.x=faces[i].x;
		faceRect.y=faces[i].y;
		faceRect.width=faces[i].width;
		faceRect.height=faces[i].height;
		Mat faceROI=colour_gray(faces[i]);
		std::cout<<"Left eye : "<<detectLeftEyeInFace(faceROI);
		std::cout<<"Right eye: "<<detectRightEyeInFace(faceROI);
		std::cout<<"Nose : "<<detectNoseInFace(faceROI);
		std::cout<<"Mouth : "<<detectMouthInFace(faceROI);
		showImage(rgb);
	}
	return true;
}

bool FaceDetector::detectLeftEyeInFace(Mat faceROI)
{
	std::vector<Rect> lefteye; 
	if( !leftEyeCascade.load( cascadeClassifier[1] ) )
	{
		std::cout<<"Error loading left eye cascade\n";
		exit(-1);
	}
	leftEyeCascade.detectMultiScale(faceROI,lefteye,1.1,2,0);
	if(lefteye.size()==0)
		return false;
	for(int i=0;i<lefteye.size();i++)
	{
		CvPoint p1={lefteye[i].x,lefteye[i].y};
		CvPoint p2={lefteye[i].x+lefteye[i].width,lefteye[i].y+lefteye[i].height};
		leftEyeRect.x=p1.x;
		leftEyeRect.y=p1.y;
		leftEyeRect.width=lefteye[i].width;
		leftEyeRect.height=lefteye[i].height;
	}
	return true;
}

bool  FaceDetector::detectRightEyeInFace(Mat faceROI)
{
	std::vector<Rect> righteye; 
	if( !rightEyeCascade.load( cascadeClassifier[2] ) )
	{
		std::cout<<"Error loading right eye cascade\n";
		exit(-1);
	}
	rightEyeCascade.detectMultiScale(faceROI,righteye,1.1,2,0);
	if(righteye.size()==0)
		return false;
	for(int i=0;i<righteye.size();i++)
	{
		CvPoint p1={righteye[i].x,righteye[i].y};
		CvPoint p2={righteye[i].x+righteye[i].width,righteye[i].y+righteye[i].height};
		rightEyeRect.x=p1.x;
		rightEyeRect.y=p1.y;
		rightEyeRect.width=righteye[i].width;
		rightEyeRect.height=righteye[i].height;
	}
	return true;
}

bool FaceDetector::detectNoseInFace(Mat faceROI)
{
	std::vector<Rect> noses; 
	if( !noseCascade.load( cascadeClassifier[3] ) )
	{
		std::cout<<"Error loading nose cascade\n";
		exit(-1);
	}
	noseCascade.detectMultiScale(faceROI,noses,1.1,2,0);
	if(noses.size()==0)
		return false;
	for(int i=0;i<noses.size();i++)
	{
		CvPoint p1={noses[i].x,noses[i].y};
		CvPoint p2={noses[i].x+noses[i].width,noses[i].y+noses[i].height};
		noseRect.x=p1.x;
		noseRect.y=p1.y;
		noseRect.width=noses[i].width;
		noseRect.height=noses[i].height;
	}
	return true;
}

bool FaceDetector::detectMouthInFace(Mat faceROI)
{
	std::vector<Rect> mouth; 
	if( !mouthCascade.load( cascadeClassifier[4] ) )
	{
		std::cout<<"Error loading mouth cascade\n";
		exit(-1);
	}
	mouthCascade.detectMultiScale(faceROI,mouth,1.1,2,0);
	if(mouth.size()==0)
		return false;
	for(int i=0;i<mouth.size();i++)
	{
		CvPoint p1={mouth[i].x,mouth[i].y};
		CvPoint p2={mouth[i].x+mouth[i].width,mouth[i].y+mouth[i].height};
		mouthRect.x=p1.x;
		mouthRect.y=p1.y;
		mouthRect.width=mouth[i].width;
		mouthRect.height=mouth[i].height;
	}
	return true;
}
CvRect FaceDetector::getFaceRect()
{
	return faceRect;
}

CvRect FaceDetector::getLeftEyeRect()
{
	return leftEyeRect;
}
CvRect FaceDetector::getRightEyeRect()
{
	return rightEyeRect;
}

CvRect FaceDetector::getNoseRect()
{
	return noseRect;
}

CvRect FaceDetector::getMouthRect()
{
	return mouthRect;
}

void FaceDetector::showImage(IplImage * rgb)
{
	cvNamedWindow("Detection");
	cvRectangle(rgb,cvPoint(faceRect.x,faceRect.y),cvPoint(faceRect.x+faceRect.width,faceRect.y+faceRect.height),CV_RGB(255,0,0),3,4);
	cvRectangle(rgb,cvPoint(leftEyeRect.x,leftEyeRect.y),cvPoint(leftEyeRect.x+leftEyeRect.width,leftEyeRect.y+leftEyeRect.height),CV_RGB(0,255,0),3,4);
	cvRectangle(rgb,cvPoint(rightEyeRect.x,rightEyeRect.y),cvPoint(rightEyeRect.x+rightEyeRect.width,rightEyeRect.y+rightEyeRect.height),CV_RGB(0,255,0),3,4);
	cvRectangle(rgb,cvPoint(noseRect.x,noseRect.y),cvPoint(noseRect.x+noseRect.width,noseRect.y+noseRect.height),CV_RGB(0,255,255),3,4);
	cvRectangle(rgb,cvPoint(mouthRect.x,mouthRect.y),cvPoint(mouthRect.x+mouthRect.width,mouthRect.y+mouthRect.height),CV_RGB(255,255,0),3,4);
	cvShowImage("Detection",rgb);
}
