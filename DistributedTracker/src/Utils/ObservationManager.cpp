#include "ObservationManager.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;
using namespace PTracking;

ObservationManager::ObservationManager(Etiseo::CameraModel* cameraModel, const Mat& H, double resolution) : cameraModel(cameraModel), H(H), resolution(resolution) {;}

vector<Rect> ObservationManager::filterBoundingBoxHorizontal(Mat& image, const Rect& boundingBox, int split, float whitePercentage)
{
	vector<Rect> boundingBoxes;
	int area, counter, height, i, minX, maxX, startBoundingBoxY, step, width;
	
	area = ((boundingBox.br().x - boundingBox.tl().x) * (boundingBox.br().y - boundingBox.tl().y)) / split;
	counter = 0;
	i = 0;
	minX = boundingBox.tl().x;
	maxX = boundingBox.br().x;
	width = boundingBox.br().x - boundingBox.tl().x;
	height = boundingBox.br().y - boundingBox.tl().y;
	step = height / split;
	
	startBoundingBoxY = boundingBox.tl().y;
	
	while (i < split)
	{
		int minY, maxY;
		
		minY = boundingBox.tl().y + (i * step);
		maxY = minY + step;
		
		counter = 0;
		
		for (int j = minY; j < maxY; ++j)
		{
			for (int k = minX; k < maxX; ++k)
			{
				if ((uchar) image.at<uchar>(j,k) == 255) ++counter;
			}
		}
		
		if ((counter / (float) area) < whitePercentage)
		{
			if ((minY - startBoundingBoxY) > 0)
			{
				Rect newBoundingBox;
				
				newBoundingBox.x = minX;
				newBoundingBox.y = startBoundingBoxY;
				newBoundingBox.width = width;
				newBoundingBox.height = minY - startBoundingBoxY;
				
				boundingBoxes.push_back(newBoundingBox);
			}
			
			startBoundingBoxY = maxY;
		}
		else
		{
			if (i == (split - 1))
			{
				Rect newBoundingBox;
				
				newBoundingBox.x = minX;
				newBoundingBox.y = startBoundingBoxY;
				newBoundingBox.width = width;
				newBoundingBox.height = maxY - startBoundingBoxY;
				
				boundingBoxes.push_back(newBoundingBox);
			}
		}
		
		++i;
	}
	
	for (vector<Rect>::iterator it = boundingBoxes.begin(); it != boundingBoxes.end(); )
	{
		for (int i = it->y; i < (it->y + it->height); ++i)
		{
			for (int j = it->x; j < (it->x + it->width); ++j)
			{
				if ((uchar) image.at<uchar>(i,j) == 255) ++counter;
			}
		}
		
		if ((counter / ((float) (it->width * it->height))) > 0.4) ++it;
		else it = boundingBoxes.erase(it);
	}
	
	return boundingBoxes;
}

vector<Rect> ObservationManager::filterBoundingBoxVertical(Mat& image, Rect& boundingBox, int split, float whitePercentage)
{
	vector<Rect> boundingBoxes;
	int area, counter, height, i, minY, maxY, startBoundingBoxX, step, width;
	
	area = ((boundingBox.br().x - boundingBox.tl().x) * (boundingBox.br().y - boundingBox.tl().y)) / split;
	counter = 0;
	i = 0;
	minY = boundingBox.tl().y ;
	maxY = boundingBox.br().y;
	width = boundingBox.br().x - boundingBox.tl().x;
	height = boundingBox.br().y - boundingBox.tl().y;
	step = width / split;
	
	startBoundingBoxX = boundingBox.tl().x;
	
	while (i < split)
	{
		int minX, maxX;
		
		minX = boundingBox.tl().x + (i * step);
		maxX = minX + step;
		
		counter = 0;
		
		for (int j = minY; j < maxY; ++j)
		{
			for (int k = minX; k < maxX; ++k)
			{
				if ((uchar) image.at<uchar>(j,k) == 255) ++counter;
			}
		}
		
		if ((counter / (float) area) < whitePercentage)
		{
			if ((minX - startBoundingBoxX) > 0)
			{
				Rect newBoundingBox;
				
				newBoundingBox.x = startBoundingBoxX;
				newBoundingBox.y = minY;
				newBoundingBox.width = minX - startBoundingBoxX;
				newBoundingBox.height = height;
				
				boundingBoxes.push_back(newBoundingBox);
			}
			
			startBoundingBoxX = maxX;
		}
		else
		{
			if (i == (split - 1))
			{
				Rect newBoundingBox;
				
				newBoundingBox.x = startBoundingBoxX;
				newBoundingBox.y = minY;
				newBoundingBox.width = maxX - startBoundingBoxX;
				newBoundingBox.height = height;
				
				boundingBoxes.push_back(newBoundingBox);
			}
		}
		
		++i;
	}
	
	for (vector<Rect>::iterator it = boundingBoxes.begin(); it != boundingBoxes.end(); )
	{
		for (int i = it->y; i < (it->y + it->height); ++i)
		{
			for (int j = it->x; j < (it->x + it->width); ++j)
			{
				if ((uchar) image.at<uchar>(i,j) == 255) ++counter;
			}
		}
		
		if ((counter / ((float) (it->width * it->height))) > 0.4) ++it;
		else it = boundingBoxes.erase(it);
	}
	
	return boundingBoxes;
}

ObjectSensorReading ObservationManager::process(Mat frame, Mat fgMask, bool visualTracker)
{
	// Filtering out shadows.
	Mat tmpBinaryImage = fgMask.clone();
	
	for (int i = 0; i < tmpBinaryImage.rows; ++i)
	{
		for (int j = 0; j < tmpBinaryImage.cols; ++j)
		{
			if (tmpBinaryImage.at<uchar>(i,j) <= 80)
			{
				tmpBinaryImage.at<uchar>(i,j) = 0;
			}
		}
	}
	
	ObjectSensorReading visualReading;
	vector<Rect> boundingBoxes;
	
	/// Kinect (People)
	//int minArea = 1000;
	//int maxArea = 20000;
	
	/// PETS-2009
	//int minArea = 370;
	//int maxArea = 10000;
	
	/// TUD-Campus
	//int minArea = 2000;
	//int maxArea = 100000;
	
	/// Edinburgh-Atrium
	int minArea = 0;
	int maxArea = 5000;
	
	vector < vector<Point> > contours;
	
	Mat element3(3,3,CV_8U,Scalar(1));
	//morphologyEx(tmpBinaryImage,tmpBinaryImage,MORPH_OPEN,element3);
	morphologyEx(tmpBinaryImage,tmpBinaryImage,MORPH_CLOSE,element3);
	
    findContours(tmpBinaryImage,contours,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
	vector<vector<Point> > contoursPoly(contours.size());
	vector<Rect> boundRect(contours.size());
	
	tmpBinaryImage = Scalar(0);
	
	vector<ObjectSensorReading::Observation> obs;
	
	for (size_t contourIdx = 0; contourIdx < contours.size(); ++contourIdx)
    {
		Moments moms = moments(Mat(contours[contourIdx]));
		double area = moms.m00;
		
        if ((area < minArea) || (area >= maxArea)) continue;
		else
		{
			approxPolyDP(Mat(contours[contourIdx]),contoursPoly[contourIdx],3,true);
			boundRect[contourIdx] = boundingRect(Mat(contoursPoly[contourIdx]));
			drawContours(tmpBinaryImage,contours,contourIdx,Scalar(255),CV_FILLED);
			
			const vector<Rect>& boundingBoxesVertical = filterBoundingBoxVertical(tmpBinaryImage,boundRect[contourIdx],8,0.2);
			
			/// Enabling for TUD-Campus.
			/*for (vector<Rect>::const_iterator it = boundingBoxesVertical.begin(); it != boundingBoxesVertical.end(); ++it)
			{
				const vector<Rect>& boundingBoxesHorizontal = filterBoundingBoxHorizontal(tmpBinaryImage,*it,8,0.1);
				
				for (vector<Rect>::const_iterator it2 = boundingBoxesHorizontal.begin(); it2 != boundingBoxesHorizontal.end(); ++it2)
				{
					boundingBoxes.push_back(*it2);
				}
			}*/
			
			for (vector<Rect>::const_iterator it = boundingBoxesVertical.begin(); it != boundingBoxesVertical.end(); ++it)
			{
				ObjectSensorReading::Observation observation;
				int counter;
				uchar hue, saturation, value;
				
				boundRect[contourIdx] = *it;
				counter = 0;
				
				const Mat& roi = frame(boundRect[contourIdx]);
				
				Mat hsvRoi;
				
				cvtColor(roi,hsvRoi,CV_BGR2HSV);
				
				for (int y = 0; y < hsvRoi.rows; ++y)
				{
					for (int x = 0; x < hsvRoi.cols; ++x)
					{
						hue = (uchar) hsvRoi.data[y * hsvRoi.step + x];
						saturation = (uchar) hsvRoi.data[y * hsvRoi.step + x + 1];
						value = (uchar) hsvRoi.data[y * hsvRoi.step + x + 2];
						
						observation.model.histograms[0][hue]++;
						observation.model.histograms[1][saturation]++;
						observation.model.histograms[2][value]++;
						
						++counter;
					}
				}
				
				for (int i = 0; i < ObjectSensorReading::Model::HISTOGRAM_VECTOR_LENGTH; ++i)
				{
					observation.model.histograms[0][i] /= counter;
					observation.model.histograms[1][i] /= counter;
					observation.model.histograms[2][i] /= counter;
				}
				
				double imageX, imageHeadX, imageY, imageHeadY;
				float barycenter, barycenterHead;
				int whiteBins[roi.cols];
				int endX, endY, index, whiteCounter;
				
				endX = boundRect[contourIdx].tl().x + roi.cols;
				endY = boundRect[contourIdx].tl().y + roi.rows;
				
				for (int i = 0; i < roi.cols; ++i)
				{
					whiteBins[i] = 0;
				}
				
				index = 0;
				whiteCounter = 0;
				
				/// Calculating center of the feet.
				for (int x = boundRect[contourIdx].tl().x; x < endX; ++x, ++index)
				{
					for (int y = (boundRect[contourIdx].tl().y + (roi.rows * 0.8)); y < endY; ++y)
					{
						if (((int) tmpBinaryImage.at<uchar>(y,x)) == 255)
						{
							whiteBins[index] += 1;
							++whiteCounter;
						}
					}
				}
				
				if (whiteCounter == 0) continue;
				
				barycenter = 0.0;
				
				for (int i = 0; i < roi.cols; ++i)
				{
					barycenter += (whiteBins[i] * i);
				}
				
				barycenter /= whiteCounter;
				
				if (barycenter == 0.0) continue;
				
				barycenter += boundRect[contourIdx].tl().x;
				
				imageX = barycenter;
				imageY = (int) boundRect[contourIdx].br().y;
				
				circle(tmpBinaryImage,Point(imageX,imageY),6,cvScalar(180),2);
				
				for (int i = 0; i < roi.cols; ++i)
				{
					whiteBins[i] = 0;
				}
				
				index = 0;
				whiteCounter = 0;
				
				/// Calculating center of the head.
				for (int x = boundRect[contourIdx].tl().x; x < endX; ++x, ++index)
				{
					for (int y = boundRect[contourIdx].tl().y; y < (endY - (roi.rows * 0.8)); ++y)
					{
						if (((int) tmpBinaryImage.at<uchar>(y,x)) == 255)
						{
							whiteBins[index] += 1;
							++whiteCounter;
						}
					}
				}
				
				if (whiteCounter == 0) continue;
				
				barycenterHead = 0.0;
				
				for (int i = 0; i < roi.cols; ++i)
				{
					barycenterHead += (whiteBins[i] * i);
				}
				
				barycenterHead /= whiteCounter;
				
				if (barycenterHead == 0.0) continue;
				
				barycenterHead += boundRect[contourIdx].tl().x;
				
				imageHeadX = barycenterHead;
				imageHeadY = (int) boundRect[contourIdx].tl().y + (((int) boundRect[contourIdx].br().y - (int) boundRect[contourIdx].tl().y) / 2);
				
				circle(tmpBinaryImage,Point(imageHeadX,imageHeadY),6,cvScalar(0),2);
				
				double imageXGoogle, imageHeadXGoogle, imageYGoogle, imageHeadYGoogle;
				
				if (visualTracker)
				{
					imageXGoogle = imageX;
					imageYGoogle = imageY;
					
					imageHeadXGoogle = imageHeadX;
					imageHeadYGoogle = imageHeadY;
					
					resolution = 1;
				}
				else
				{
					imageXGoogle = (H.at<double>(0,0) * imageX + H.at<double>(0,1) * imageY + H.at<double>(0,2)) / (H.at<double>(2,0) * imageX + H.at<double>(2,1) * imageY + H.at<double>(2,2));
					imageYGoogle = (H.at<double>(1,0) * imageX + H.at<double>(1,1) * imageY + H.at<double>(1,2)) / (H.at<double>(2,0) * imageX + H.at<double>(2,1) * imageY + H.at<double>(2,2));
					
					imageHeadXGoogle = (H.at<double>(0,0) * imageHeadX + H.at<double>(0,1) * imageHeadY + H.at<double>(0,2)) / (H.at<double>(2,0) * imageHeadX + H.at<double>(2,1) * imageHeadY + H.at<double>(2,2));
					imageHeadYGoogle = (H.at<double>(1,0) * imageHeadX + H.at<double>(1,1) * imageHeadY + H.at<double>(1,2)) / (H.at<double>(2,0) * imageHeadX + H.at<double>(2,1) * imageHeadY + H.at<double>(2,2));
				}
				
				rectangle(tmpBinaryImage,boundRect[contourIdx].tl(),boundRect[contourIdx].br(),CV_RGB(190,190,190),1,8,0);
				
				observation.observation.rho = sqrt(pow(imageXGoogle * resolution,2) + pow(imageYGoogle * resolution,2));
				observation.observation.theta = atan2(imageYGoogle * resolution,imageXGoogle * resolution);
				observation.head.x = imageHeadXGoogle * resolution;
				observation.head.y = imageHeadYGoogle * resolution;
				observation.model.barycenter = barycenter;
				observation.model.boundingBox = make_pair(PTracking::Point2f(boundRect[contourIdx].tl().x - barycenter,-roi.rows),PTracking::Point2f(roi.cols + boundRect[contourIdx].tl().x - barycenter,0));
				observation.model.height = (int) boundRect[contourIdx].br().y - (int) boundRect[contourIdx].tl().y;
				observation.model.width = (int) boundRect[contourIdx].br().x - (int) boundRect[contourIdx].tl().x;
				
				obs.push_back(observation);
			}
		}
	}
	
	visualReading.setObservations(obs);
	visualReading.setObservationsAgentPose(Point2of(0.0,0.0,0.0));
	
	imshow("Foreground Model",tmpBinaryImage);
	
	return visualReading;
}
