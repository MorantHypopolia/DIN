#pragma once

#include <opencv2/core/core.hpp>
#include <libfreenect.hpp>
#include <vector>

#define FREENECT_FRAME_W			640
#define FREENECT_FRAME_H			480
#define FREENECT_FRAME_PIX			(FREENECT_FRAME_H * FREENECT_FRAME_W)
#define FREENECT_VIDEO_RGB_SIZE		(FREENECT_FRAME_PIX * 3)

class Mutex_v
{
	private:
		pthread_mutex_t m_mutex;
		
	public:
		Mutex_v()
		{
			pthread_mutex_init( &m_mutex,0);
		}
		
		void lock()
		{
			pthread_mutex_lock( &m_mutex );
		}
		
		void unlock()
		{
			pthread_mutex_unlock( &m_mutex );
		}
		
		class ScopedLock
		{
			Mutex_v& _mutex;
			
			public:
				ScopedLock(Mutex_v& mutex) : _mutex(mutex)
				{
					_mutex.lock();
				}
				
				~ScopedLock(){
					_mutex.unlock();
				}
		};
};

class MyFreenectDevice : public Freenect::FreenectDevice
{
	private:
		std::vector<uint16_t> m_gamma;
		std::vector<uint8_t> m_buffer_depth, m_buffer_rgb;
		Mutex_v m_depth_mutex, m_rgb_mutex;
		cv::Mat depthMat, ownMat, rgbMat;
		bool m_new_depth_frame, m_new_rgb_frame;
		
	public:
		MyFreenectDevice(freenect_context* _ctx, char* _serial);
		
		void DepthCallback(void* _depth, uint32_t timestamp);
		bool getDepth(cv::Mat& output);
		bool getVideo(cv::Mat& output);
		void VideoCallback(void* _rgb, uint32_t timestamp);
};
