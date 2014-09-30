#include "MyFreenectDevice.h"
#include <opencv/cvaux.h>

using namespace std;
using namespace cv;

MyFreenectDevice::MyFreenectDevice(freenect_context* _ctx, char* _serial) : Freenect::FreenectDevice(_ctx,_serial), m_gamma(2048), m_buffer_depth(FREENECT_DEPTH_11BIT), m_buffer_rgb(FREENECT_VIDEO_RGB),
																			depthMat(Size(640,480),CV_16UC1), ownMat(Size(640,480),CV_8UC3,Scalar(0)), rgbMat(Size(640,480),CV_8UC3,Scalar(0)),
																			m_new_depth_frame(false), m_new_rgb_frame(false)
{
	for (unsigned int i = 0 ; i < 2048 ; i++)
	{
		float v = i / 2048.0;
		
		v = pow(v,3) * 6;
		
		m_gamma[i] = v * 6 * 256;
	}
}

void MyFreenectDevice::VideoCallback(void* _rgb, uint32_t)
{
	Mutex_v::ScopedLock lock(m_rgb_mutex);
	
	uint8_t* rgb = static_cast<uint8_t*>(_rgb);
	copy(rgb,rgb + getVideoBufferSize(),m_buffer_rgb.begin());
	memcpy(rgbMat.data,rgb,FREENECT_VIDEO_RGB_SIZE);
	
	m_new_rgb_frame = true;
}

void MyFreenectDevice::DepthCallback(void* _depth, uint32_t)
{
	Mutex_v::ScopedLock lock(m_depth_mutex);
	
	uint16_t* depth = static_cast<uint16_t*>(_depth);
	
	int offset = 0;
	
	for (int i = 0; i < FREENECT_FRAME_H; ++i)
	{
		for (int j = 0; j < FREENECT_FRAME_W; ++j)
		{
			memcpy(&depthMat.data[offset],&depth[i * FREENECT_FRAME_W + j],sizeof(short));
			offset += sizeof(short);
		}
	}
	
	m_new_depth_frame = true;
}

bool MyFreenectDevice::getVideo(Mat& output)
{
	Mutex_v::ScopedLock lock(m_rgb_mutex);
	
	if (m_new_rgb_frame)
	{
		cvtColor(rgbMat,output,CV_RGB2BGR);
		m_new_rgb_frame = false;
		
		return true;
	}
	else return false;
}

bool MyFreenectDevice::getDepth(Mat& output)
{
	Mutex_v::ScopedLock lock(m_depth_mutex);
	
	if (m_new_depth_frame)
	{
		depthMat.copyTo(output);
		m_new_depth_frame = false;
		
		return true;
	}
	else return false;
}
