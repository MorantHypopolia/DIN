#pragma once

#include "Kinect.h"
#include "MyFreenectDevice.h"

class KinectWriter : public Kinect
{
	private:
		MyFreenectDevice* device;
		Freenect::Freenect freenect;
		unsigned long initial_tick_count;
		int cnt_fails;
		char* serial;
		bool view;
		
		inline double getTimestamp();
		
	public:
		KinectWriter() {;}
		KinectWriter(const std::string&,const bool,char*);
		
		virtual ~KinectWriter();
		
		virtual bool getData();
};
