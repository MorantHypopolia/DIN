#pragma once

#include <opencv2/core/core.hpp>

class Kinect
{
	public:
		cv::Mat depth, depthMat16bit, frame;
		std::string sequenceName;
		unsigned int frameCounter;
		int width, height;
		
		Kinect() {;}
		
		virtual ~Kinect() {;}
		
		virtual bool getData() = 0;
};
