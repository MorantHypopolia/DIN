#pragma once

#include <opencv2/core/core.hpp>
#include <CameraModel/cameraModel.h>
#include <CameraModel/xmlUtil.h>
#include <Core/Filters/ObjectSensorReading.h>

class ObservationManager
{
	private:
		Etiseo::CameraModel* cameraModel;
		cv::Mat H;
		double resolution;
		
		std::vector<cv::Rect> filterBoundingBoxHorizontal(cv::Mat&,const cv::Rect&,int,float);
		std::vector<cv::Rect> filterBoundingBoxVertical(cv::Mat&,cv::Rect&,int,float);
		
	public:
		ObservationManager(Etiseo::CameraModel*,const cv::Mat&,double);
		
		inline Etiseo::CameraModel* getCameraModel() { return cameraModel; }
		PTracking::ObjectSensorReading process(cv::Mat,cv::Mat,bool);
};
