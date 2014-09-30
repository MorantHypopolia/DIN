#pragma once

#include "Kinect.h"
#include <sstream>
#include <vector>

class KinectReader : public Kinect
{
	private:
		std::vector<std::string> filenameRGB;
		std::stringstream frameNumber;
		std::string baseName;
		double delay;
		int fps;
		
	public:
		KinectReader() {;}
		KinectReader(const std::string&);
		
		virtual ~KinectReader();
		
		virtual bool getData();
};
