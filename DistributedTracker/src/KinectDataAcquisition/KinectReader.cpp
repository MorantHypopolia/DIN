#include "KinectReader.h"
#include <opencv/cvaux.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dirent.h>
#include <unistd.h>

using namespace std;
using namespace cv;

KinectReader::KinectReader(const string& directoryName)
{
	this->sequenceName = directoryName;
	
	struct dirent **direct;
	int n = scandir(sequenceName.c_str(),&direct,0,alphasort);
	
	if (n == 0)
	{
		cout << "Unable to open directory!" << endl;
		
		exit(-1);
		
	}
	else
	{
		for (int i = 0; i < n; ++i)
		{
			if (strstr(direct[i]->d_name,".jpg")) filenameRGB.push_back(string(direct[i]->d_name));
		}
	}
	
	if (filenameRGB.size() == 0)
	{
		cout << "Error: No files found!" << endl;
		
		exit(-1);
	}
	
	int pos = filenameRGB[0].find("_RGB_");
	
	baseName = filenameRGB[0].substr(0, pos);
	
	cout << "basename: " << baseName << endl;
	
	frameCounter = 0;
	delay = (1 / 30.0) * 2000000;
}

KinectReader::~KinectReader() {;}

bool KinectReader::getData()
{
	if (frameCounter == filenameRGB.size()) return false;
	
	stringstream searchTime;
	
	searchTime << "_" << frameCounter << "_";
	
	int idx = -1;
	
	for (unsigned int i = 0; i < filenameRGB.size(); ++i)
	{
		if (strstr(filenameRGB[i].c_str(), searchTime.str().c_str()) != 0)
		{
			idx = i;
			
			break;
		}
	}
	
	cout << filenameRGB[idx] << endl;
	
	stringstream name;
	
	name << sequenceName << "/" << filenameRGB[idx];
	
	frame = imread(name.str());
	frameNumber.str(string());
	frameNumber << frameCounter;
	
	rectangle(frame,Point(10,2),Point(100,20),Scalar(255,255,255),-1);
	putText(frame,frameNumber.str().c_str(),Point(15,15),CV_FONT_NORMAL,0.5,Scalar(0,0,0));
	
	name.str(string());
	name << sequenceName << "/" << baseName << "_DEPTH_" << frameCounter << ".png" ;
	
	cout << name.str() << endl;
	
	depthMat16bit = imread(name.str(),CV_16UC1);
	
	Mat adjMap;
	
	depth = Mat::zeros(depthMat16bit.size(),CV_8UC3);
	depthMat16bit.convertTo(adjMap,CV_8UC1,255.0 / 2048.0);
	applyColorMap(adjMap,depth,COLORMAP_JET);
	
	++frameCounter;
	
	usleep(delay);
	
	return true;
}
