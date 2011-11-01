/*
 * kinectopencv.cpp
 *
 *  Created on: 11 Dec 2010
 *      Author: Murilo Fernandes Martins
 *      Email: muhrix@gmail.com
 *
 */

#include <iostream>

#include <cv.h>
#include <cxcore.h>
#include <cvaux.h>
#include <highgui.h>
#include "KinectOpenCV.h"
#include "asmfitting.h"
#include "vjfacedetect.h"
#include "FaceDetector.h"

void onMouse(int event, int x, int y, int flags, void* params) {
	KinectOpenCV* dev = static_cast<KinectOpenCV*>(params);
	FILE *fp;
	fp=fopen("depth.txt","w");
	fprintf(fp,"Starting new\n\n");
	int i,j;
	for(i=0;i<640;i++)
	{
		for(j=1;j<480;j++)
		{	fprintf(fp,"%.2f",dev->GetDepthInMetres(i,j));
			fprintf(fp,"  ");
		}
		fprintf(fp,"\n");
	}
	fclose(fp);
	double dist = dev->GetDepthInMetres(x, y);

	if( dist != -1 ) {
		std::cout << "Depth(" << x << ", " << y << "): " << dist << " metres" << std::endl;
	}
	else {
		std::cout << "Depth(" << x << ", " << y << "): cannot compute value" << std::endl;
	}
	std::cout.flush();
}


asmfitting fit_asm;
int loadAAMFiles()
{
	char* model_name = "C:\\For_Shivani\\FaceApp\\asmlibrary-win-6.0\\example\\my68-1d.amf";
	char* cascade_name = "C:\\For_Shivani\\FaceApp\\asmlibrary-win-6.0\\example\\haarcascade_frontalface_alt2.xml";
	if(fit_asm.Read(model_name) == false)
		return -1;
	
	if(init_detect_cascade(cascade_name) == false)
		return -1;
	return 1;
}

bool flag = false;
int j = 0;
asm_shape shape, detshape;
int n_iteration=24;

void aamTest(IplImage* image)
{	
		cvNamedWindow("ASM-Search",1);	
		CvPoint pt1,pt2;
			if(flag == false)
			{
				//Firstly, we detect face by using Viola_jones haarlike-detector
				flag = detect_one_face(detshape, image);
				
				//Secondly, we initialize shape from the detected box
				if(flag)
				{
					pt1.x=detshape.MinX();
				pt1.y=detshape.MinY();
				pt2.x=detshape.MaxX();
				pt2.y=detshape.MaxY();
				std::cout<<"Face point 1  "<<detshape.MinX()<<"  "<<detshape.MinY();
				std::cout<<"Face point 2  "<<pt2.x<<"  "<<pt2.y;
				cvRectangle(image,pt1,pt2,CV_RGB(0,0,255),3,4,0);
					InitShapeFromDetBox(shape, detshape, fit_asm.GetMappingDetShape(), fit_asm.GetMeanFaceWidth());
					j ++;
				}
				else 
					goto show2;
			}
			
			//Thirdly, we do image alignment 
			flag = fit_asm.ASMSeqSearch(shape, image, j, true, n_iteration);
			
			//If success, we draw and show its result
			if(flag) {
				fit_asm.Draw(image, shape);
				asm_shape temp_shape = fit_asm.GetMappingDetShape();
				cout<<"--------------------------"<<endl;
				cout<<"NPoints"<<temp_shape.NPoints()<<endl;
				for(int k =0;k<temp_shape.NPoints();k++)
				{
					cout<<k<<"=["<<temp_shape[k].x<<","<<temp_shape[k].y<<"]"<<endl;
				}
				cout<<"--------------------------"<<endl;
//				cvWaitKey(0);
			}
show2:
			cvShowImage("ASM-Search", image);		
}

int main(int argc, char *argv[]) {

	
	std::cout<<"Kinect test"<<std::endl;
	FaceDetector f;
	IMU imu;
	KinectOpenCV kinect(0);
	char key;
	IplImage *camera = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 3);
	IplImage *depth = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 3);
	cvZero(camera);
	cvZero(depth);

	cvNamedWindow("Camera", 1);
	cvNamedWindow("Depth", 1);
	cvShowImage("Camera", camera);
	cvShowImage("Depth", depth);

	cvSetMouseCallback("Depth", onMouse, (void*)&kinect);

	cvWaitKey(10);

	kinect.StartKinect();


	//----
	int x=loadAAMFiles();
	bool filesLoaded = true;
	if(x == -1) 
	{cout<<"Unable to load Required model files and cascade"<<endl;
	filesLoaded = false;
//	cvWaitKey(0);
//	exit(0);
	}
	//---

	for(;;) {
		key = (cvWaitKey(5) & 255);
		if( key == 27 ) {
			break;
		}
		if( key == 'w' ) {
			kinect.SetTiltAngle(20);
		}
		if( key == 'x' ) {
			kinect.SetTiltAngle(-20);
		}
		if( key == 's' ) {
			kinect.SetTiltAngle(0);
		}
		//imu = kinect.GetIMUData();
		//std::cout << "IMU: " << imu.x << "\t" << imu.y << "\t" << imu.z << std::endl;
		//std::cout.flush();

		kinect.GetColourImage(camera);

		cvShowImage("Camera", camera);

		kinect.GetDepthImage(depth);
		cvShowImage("Depth", depth);
		f.detectFaceInImage(camera);
		if(filesLoaded)
		aamTest(camera);
	}

	kinect.StopKinect();

	cvDestroyWindow("Camera");
	cvDestroyWindow("Depth");
	cvReleaseImage(&camera);
	cvReleaseImage(&depth);
	
	return 0;
}

