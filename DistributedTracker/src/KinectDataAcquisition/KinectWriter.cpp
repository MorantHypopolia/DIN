#include "KinectWriter.h"
#include <opencv/cvaux.h>
#include <opencv2/highgui/highgui.hpp>
#include <Manfield/utils/debugutils.h>
#include <unistd.h>

using namespace std;
using namespace cv;

KinectWriter::KinectWriter(const string& sequenceName, bool view, char* serial)
{
	this->sequenceName = sequenceName;
	this->view = view;
	
	width = 640;
	height = 480;
	
	depthMat16bit = Mat(Size(width,height),CV_16UC1);
	depth = Mat(Size(width,height),CV_8UC3);
	frame = Mat(Size(width,height),CV_8UC3,Scalar(0));
	
	try
	{
		device = &freenect.createDevice<MyFreenectDevice>(serial);
	}
	catch (const runtime_error& e)
	{
		ERR(e.what() << ". Exiting..." << endl);
		
		exit(-1);
	}
	
	DEBUG("Device " << serial << ": Opened!" << endl);
	
	initial_tick_count = getTimestamp();
	
	device->startVideo();
	device->startDepth();
	frameCounter = 0;
	
	this->serial = serial;
	
	cnt_fails = 0;
}

KinectWriter::~KinectWriter()
{
	device->stopVideo();
	device->stopDepth();
	freenect.deleteDevice(serial);
	
	delete device;
}

inline double KinectWriter::getTimestamp()
{
	struct timeval tv;
	gettimeofday(&tv,0);
	
	return (unsigned long) (tv.tv_sec % 86400) * 1000 + (unsigned long) tv.tv_usec / 1000;
}

bool KinectWriter::getData()
{
	if (device->getVideo(frame))
	{
		device->getDepth(depthMat16bit);
		
		if (!view)
		{
			stringstream nameRGB;
			stringstream nameDepth;
			
			nameRGB << "rgb_" << frameCounter << ".jpg";
			nameDepth << "depth_" << frameCounter << ".pgm";
			
			imwrite(nameRGB.str().c_str(),frame);
			imwrite(nameDepth.str().c_str(),depthMat16bit);
			
			Mat gray;
			cvtColor(frame,gray,CV_RGB2GRAY);
			
			nameDepth.str(string());
			nameDepth << "rgb_" << frameCounter << ".pgm";
			
			imwrite(nameDepth.str().c_str(),gray);
		}
		
		Mat adjMap;
		
		depthMat16bit.convertTo(adjMap,CV_8UC1,255.0 / 2048.0);
		applyColorMap(adjMap,depth,COLORMAP_JET);
		
		++frameCounter;
		cnt_fails = 0;
		
		return true;
	}
	else if (view && cnt_fails < 5)
	{
		++cnt_fails;
		
		return true;
	}
	else
	{
		static bool isFirstTime = true;
		
		if (isFirstTime)
		{
			WARN("Failed to read images from the kinect. I'm trying to restart everything before giving up..." << endl);
			
			isFirstTime = false;
			
			device->stopVideo();
			device->stopDepth();
			
			freenect.deleteDevice(serial);
			
			INFO("Device " << serial << ": Closed!" << endl);
			
			try
			{
				device = &freenect.createDevice<MyFreenectDevice>(serial);
			}
			catch (const runtime_error& e)
			{
				ERR(e.what() << ". Exiting..." << endl);
				
				exit(-1);
			}
			
			DEBUG("Device " << serial << ": Opened!" << endl);
			
			initial_tick_count = getTimestamp();
			
			device->startVideo();
			device->startDepth();
			frameCounter = 0;
			
			this->serial = serial;
			
			cnt_fails = 0;
			
			return true;
		}
		
		ERR("The restart didn't succeed... I'm definitely giving up" << endl);
		
		return false;
	}
}
