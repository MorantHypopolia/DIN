/*
 *  IMBS Background Subtraction Library 
 *
 *  This file main.cpp contains an example of usage for
 *  IMBS algorithm described in
 *  D. D. Bloisi and L. Iocchi
 *  "Independent Multimodal Background Subtraction"
 *  In Proc. of the Third Int. Conf. on Computational Modeling of Objects
 *  Presented in Images: Fundamentals, Methods and Applications, pp. 39-44, 2012.
 *  Please, cite the above paper if you use IMBS.
 *  
 *  This software is provided without any warranty about its usability. 
 *  It is for educational purposes and should be regarded as such.
 *
 *  Written by Domenico D. Bloisi
 *
 *  Please, report suggestions/comments/bugs to
 *  domenico.bloisi@gmail.com
 *
 */

#include <inttypes.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>

//OpenCV
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// IMBS
#include "IMBS/imbs.hpp"
#include "Utils/ObservationManager.h"

// KinectDataAcquisition
#include "KinectDataAcquisition/KinectWriter.h"

//PTracking
#include <PTracker/PTracker.h>
#include <Utils/Point2f.h>
#include <Utils/Point2of.h>
#include <Manfield/configfile/configfile.h>
#include <sys/stat.h>

// Uncomment if you want to write the results in a xml file.
//#define RESULTS_ENABLED

// Uncomment if you want to visualize the tracklets of the tracked objects.
//#define VISUALIZE_TRACKLETS

// Uncomment if you want to print the current frame rate.
//#define FRAME_RATE

using namespace cv;
using namespace std;
using namespace PTracking;
using GMapping::ConfigFile;

static const int MAXIMUM_NUMBER_OF_MISSING_FRAMES = 200;

Mat frame;
Mat fgMask;
Mat bgImage;
Mat H(3,3,DataType<double>::type);
Mat Hinv(3,3,DataType<double>::type);
Mat Pinv(4,4,DataType<double>::type);
Mat Kinect2World(4,4,DataType<double>::type);
double fps, resolution;
int agentId = -100, keyboard;
bool slow;
bool opticalTracker;

string load_bg_filename;
string save_bg_filename;
bool load_bg = false;
bool save_bg = false;

map<int,pair<int,pair<int,int> > > colorMap;

vector<string> serialList;
int idx = 0;
bool view;

void help()
{
	cout << "--------------------------------------------------------------------------" << endl
		 << "IMBS Background Subtraction Library "                                       << endl
		 << "This file main.cpp contains an example of usage for"                        << endl
		 << "IMBS algorithm described in"                                                << endl
		 << "D. D. Bloisi and L. Iocchi"                                                 << endl
		 << "\"Independent Multimodal Background Subtraction\""                          << endl
		 << "In Proc. of the Third Int. Conf. on Computational Modeling of Objects"      << endl
		 << "Presented in Images: Fundamentals, Methods and Applications,"               << endl
		 << "pp. 39-44, 2012."                                                           << endl
		 << endl
		 << "written by Domenico D. Bloisi"                                              << endl
		 << "domenico.bloisi@gmail.com"                                                  << endl
		 << "--------------------------------------------------------------------------" << endl
		 << "You can process both videos (-vid) and images (-img)."                      << endl
		 << endl
		 << "Usage:"                                                                     << endl
		 << "imbs {-vid <video filename>|-img <image filename> [-fps <value>]}"          << endl
		 << "for example: imbs -vid video.avi"                                           << endl
		 << "or: imbs -img /data/images/1.png"                                           << endl
		 << "or: imbs -img /data/images/1.png -fps 7"                                    << endl
		 << "--------------------------------------------------------------------------" << endl
		 << endl;
}

void configure(const string& filename, const string& homography, const string& dataset, bool isKinect = false)
{
	ConfigFile fCfg;
	string key, section;
	
	if (!fCfg.read(filename))
	{
		ERR("Error reading file '" << filename << "'. Exiting..." << endl);
		
		exit(-1);
	}
	
	try
	{
		section = "parameters";
		key = "opticalTracker";
		
		opticalTracker = fCfg.value(section,key);
	}
	catch (...)
	{
		ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
		
		exit(-1);
	}
	
	if ((!opticalTracker) || isKinect)
	{
		if (!fCfg.read(homography))
		{
			ERR("Error reading file '" << homography << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
		try
		{
			section = "parameters";
			key = "resolution";
			
			resolution = fCfg.value(section,key);
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
		stringstream s;
		
		if (isKinect) s << "Kinect_" << agentId;
		else s << dataset << "_" << agentId;
		
		section = s.str();
		
		try
		{
			key = "H00";
			H.at<double>(0,0) = fCfg.value(section,key);
			
			key = "H01";
			H.at<double>(0,1) = fCfg.value(section,key);
			
			key = "H02";
			H.at<double>(0,2) = fCfg.value(section,key);
			
			key = "H10";
			H.at<double>(1,0) = fCfg.value(section,key);
			
			key = "H11";
			H.at<double>(1,1) = fCfg.value(section,key);
			
			key = "H12";
			H.at<double>(1,2) = fCfg.value(section,key);
			
			key = "H20";
			H.at<double>(2,0) = fCfg.value(section,key);
			
			key = "H21";
			H.at<double>(2,1) = fCfg.value(section,key);
			
			key = "H22";
			H.at<double>(2,2) = fCfg.value(section,key);
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
		s.clear();
		s.str("");
		
		if (isKinect) s << "Kinect_" << agentId << "-inverse";
		else s << dataset << "_" << agentId << "-inverse";
		
		section = s.str();
		
		try
		{
			key = "H00";
			Hinv.at<double>(0,0) = fCfg.value(section,key);
			
			key = "H01";
			Hinv.at<double>(0,1) = fCfg.value(section,key);
			
			key = "H02";
			Hinv.at<double>(0,2) = fCfg.value(section,key);
			
			key = "H10";
			Hinv.at<double>(1,0) = fCfg.value(section,key);
			
			key = "H11";
			Hinv.at<double>(1,1) = fCfg.value(section,key);
			
			key = "H12";
			Hinv.at<double>(1,2) = fCfg.value(section,key);
			
			key = "H20";
			Hinv.at<double>(2,0) = fCfg.value(section,key);
			
			key = "H21";
			Hinv.at<double>(2,1) = fCfg.value(section,key);
			
			key = "H22";
			Hinv.at<double>(2,2) = fCfg.value(section,key);
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
	}
}

void processVideo(char* videoFilename, const string& dataset, ObservationManager* observationManager)
{
	VideoCapture capture(videoFilename);
	
	if (!capture.isOpened())
	{
		cerr << "Unable to open video file: " << videoFilename << endl;
		
		exit(EXIT_FAILURE);
	}
	
	BackgroundSubtractorIMBS* pIMBS;
	fps = capture.get(5);
	
	pIMBS = new BackgroundSubtractorIMBS(fps);
	
	if (load_bg)
	{
		cout<<"loading initial background...";
		cout.flush();
		
		if (pIMBS->loadBg(load_bg_filename.c_str())) cout<<"done"<<endl;
		else
		{
			cerr << "Unable to open file "<<load_bg_filename<<endl;
			cerr<<"Process terminated."<<endl;
			
			exit(EXIT_FAILURE);
		}
	}
	
	if (save_bg)
	{
		cout<<"Initial background will be saved as "<<save_bg_filename<<endl;
		
		pIMBS->saveBg(&save_bg_filename);
	}
	
	PTracker* pTracker;
	
	pTracker = new PTracker(agentId);
	
#ifdef RESULTS_ENABLED
	ofstream results;
	
	if (agentId == 1)
	{
		struct stat temp;
		
		if (stat("../results",&temp) == -1)
		{
			mkdir("../results",0775);
		}
		
		stringstream s;
		
		s << "../results/PTracker-" << dataset << ".xml";
		
		results.open(s.str().c_str());
		
		results << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << endl;
		results << "<dataset>" << endl;
	}
	
	int resultsIteration;
	
	if (agentId == 1)
	{
		resultsIteration = 0;
	}
#endif
	
	while ((char) keyboard != 'q' && (char) keyboard != 27)
	{
		if (!capture.read(frame))
		{
			cerr << "Unable to read next frame." << endl;
			cerr << "Exiting..." << endl;
			
			exit(EXIT_FAILURE);
		}
		
		pIMBS->apply(frame, fgMask);
		pIMBS->getBackgroundImage(bgImage);
		
		if (observationManager != 0)
		{
			ObjectSensorReading visualReading = observationManager->process(frame,fgMask,opticalTracker);
			
			Mat scene;
			
			scene = frame.clone();
			
			if (visualReading.getObservations().size() > 0)
			{
				pTracker->exec(visualReading);
				
				const pair<map<int,pair<pair<ObjectSensorReading::Observation,PTracking::Point2f>,pair<string,int> > >,map<int,pair<ObjectSensorReading::Observation,PTracking::Point2f> > >& estimatedTargetModelsWithIdentity = pTracker->getAgentEstimations();
				
#ifdef RESULTS_ENABLED
				if (agentId == 1)
				{
					if (estimatedTargetModelsWithIdentity.first.size() > 0)
					{
						results << "   <frame number=\"" << resultsIteration++ << "\">" << endl;
						results << "      <objectlist>" << endl;
					}
				}
#endif
				
				for (map<int,pair<pair<ObjectSensorReading::Observation,PTracking::Point2f>,pair<string,int> > >::const_iterator it = estimatedTargetModelsWithIdentity.first.begin();
																																 it != estimatedTargetModelsWithIdentity.first.end(); ++it)
				{
					cv::Point2f p;
					
					if (opticalTracker) resolution = 1;
					
					p.x = it->second.first.first.observation.getCartesian().x / resolution;
					p.y = it->second.first.first.observation.getCartesian().y / resolution;
					
					double imageX, imageY;
					
					if (opticalTracker)
					{
						imageX = p.x;
						imageY = p.y;
					}
					else continue;
					
					map<int,pair<int,pair<int,int> > >::iterator colorTrack = colorMap.find(it->first);
					
					rectangle(scene,cvPoint(imageX - (it->second.first.first.model.width / 2), imageY - it->second.first.first.model.height),
							  cvPoint(imageX + (it->second.first.first.model.width / 2), imageY),
							  cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second), 3);
					
					stringstream text;
					
					text << it->first;
					
					int fontFace = FONT_HERSHEY_SIMPLEX;
					double fontScale = 1;
					int thickness = 3;
					Point textOrg(imageX - (it->second.first.first.model.width / 2), imageY - it->second.first.first.model.height - 7);
					putText(scene, text.str(), textOrg, fontFace, fontScale, Scalar::all(0), thickness,8);
					
					fontScale = 1;
					thickness = 2;
					Point textOrg2(imageX - (it->second.first.first.model.width / 2) + 2, imageY - it->second.first.first.model.height - 9);
					putText(scene, text.str(), textOrg2, fontFace, fontScale, Scalar::all(255), thickness,8);
					
#ifdef RESULTS_ENABLED
					if (agentId == 1)
					{
						results << "         <object id=\"" << it->first << "\">" << endl;
						results << "            <box h=\"" << it->second.first.first.model.height << "\" w=\"" << it->second.first.first.model.width << "\" xc=\"" << imageX
								<< "\" yc=\"" << (imageY - (it->second.first.first.model.height / 2)) << "\"/>" << endl;
						results << "         </object>" << endl;
					}
#endif
				}
				
#ifdef RESULTS_ENABLED
				if (agentId == 1)
				{
					if (estimatedTargetModelsWithIdentity.first.size() > 0)
					{
						results << "      </objectlist>" << endl;
						results << "   </frame>" << endl;
					}
				}
#endif
			}
			
			imshow("PTracking",scene);
		}
		
		stringstream ss;
		
		rectangle(frame,Point(10,2),Point(100,20),Scalar(255,255,255),-1);
		
		ss << capture.get(1);
		
		string frameNumberString = ss.str();
		
		putText(frame,frameNumberString.c_str(),Point(15, 15),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0));
		
		//imshow("Frame", frame);
		//imshow("FG Mask", fgMask);
		//imshow("BG Model", bgImage);
		
		keyboard = waitKey(30);
	}
	
	capture.release();
}

void processImages(char* firstFrameFilename, const string& dataset, ObservationManager* observationManager)
{
	Mat planarView, planarViewTemplate;
	string nextFrameFilename;
	
	frame = imread(firstFrameFilename);
	
	if (!frame.data)
	{
		cerr << "Unable to open first image frame: " << firstFrameFilename << endl;
		
		exit(EXIT_FAILURE);
	}
	
	string frameBaseName, frameDigits, extension;
	
	frameBaseName = string(firstFrameFilename).substr(0,string(firstFrameFilename).rfind('_') + 1);
	frameDigits = string(firstFrameFilename).substr(string(firstFrameFilename).rfind('_') + 1,string(firstFrameFilename).rfind('.') - string(firstFrameFilename).rfind('_') - 1);
	extension = string(firstFrameFilename).substr(string(firstFrameFilename).rfind('.'));
	
	BackgroundSubtractorIMBS* pIMBS;
	
	pIMBS = new BackgroundSubtractorIMBS(fps);
	
	if (load_bg)
	{
        cout << "Loading initial background...";
		cout.flush();
		
		if (pIMBS->loadBg(load_bg_filename.c_str())) cout << "done" << endl;
		else
		{
			cerr << "Unable to open file " << load_bg_filename << endl;
			cerr << "Process terminated." << endl;
			
			exit(EXIT_FAILURE);
		}
	}
	
	if (save_bg)
	{
		cout << "Initial background will be saved as " << save_bg_filename << endl;
		
		pIMBS->saveBg(&save_bg_filename);
	}
	
	PTracker* pTracker;
	
	pTracker = new PTracker(agentId,string(getenv("PTracking_ROOT")) + string("/../config/") + dataset + string("/parameters.cfg"));
	
#ifdef RESULTS_ENABLED
	ofstream results;
	
	if (agentId == 1)
	{
		struct stat temp;
		
		if (stat("../results",&temp) == -1)
		{
			mkdir("../results",0775);
		}
		
		stringstream s;
		
		s << "../results/PTracker-" << dataset << ".xml";
		
		results.open(s.str().c_str());
		
		results << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << endl;
		results << "<dataset>" << endl;
	}
	
	int resultsIteration;
	
	if (agentId == 1)
	{
		resultsIteration = 0;
	}
#endif
	
	string fn(firstFrameFilename);
	
	int frameNumber = atoi(frameDigits.c_str());
	int width, height;
	
	planarViewTemplate = imread((string("../CameraView/") + dataset + string("/") + dataset + string("-PlanarView.png")).c_str());
	
	int offset = 24;
	
	namedWindow(dataset + string(" Planar View"));
	moveWindow(dataset + string(" Planar View"),frame.cols - (planarViewTemplate.cols / 2),offset);
	
	namedWindow("Foreground Model");
	moveWindow("Foreground Model",0,planarViewTemplate.rows + 2 + (offset * 2));
	
	stringstream s;
	
	s << "PTracking - View " << agentId;
	
	namedWindow(s.str());
	moveWindow(s.str(),frame.cols + 1,planarViewTemplate.rows + 2 + (offset * 2));
	
#ifdef VISUALIZE_TRACKLETS
	map<int,vector<PTracking::Point2f> > targetHistory;
	
	Mat googleFrame = imread((string("../CameraView/") + dataset + string("/") + dataset + string("-PlanarView.png")).c_str());
	Mat googleScene;
	
	namedWindow(s.str() + " (Tracklets)");
	moveWindow(s.str() + " (Tracklets)",(frame.cols * 2) + 1,planarViewTemplate.rows + 2 + (offset * 2));
	imshow(s.str() + " (Tracklets)",frame);
	
	if (agentId == 1)
	{
		namedWindow(dataset + string(" Planar View (Tracklets)"));
		moveWindow(dataset + string(" Planar View (Tracklets)"),(frame.cols * 2) - (googleFrame.cols / 2),offset);
		imshow(dataset + string(" Planar View (Tracklets)"),googleFrame);
	}
#endif
	
	while ((char)keyboard != 'q' && (char)keyboard != 27)
	{
#ifdef FRAME_RATE
		Timestamp initialTimestamp;
#endif
		
		pIMBS->apply(frame,fgMask);
		pIMBS->getBackgroundImage(bgImage);
		
		if (observationManager != 0)
		{
			ObjectSensorReading visualReading = observationManager->process(frame,fgMask,opticalTracker);
			
			Mat scene;
			
			scene = frame.clone();
			planarView = planarViewTemplate.clone();
			
#ifdef VISUALIZE_TRACKLETS
			Mat sceneMerged;
			
			sceneMerged = frame.clone();
#endif
			
			pTracker->exec(visualReading);
			
			const pair<map<int,pair<pair<ObjectSensorReading::Observation,PTracking::Point2f>,pair<string,int> > >,map<int,pair<ObjectSensorReading::Observation,PTracking::Point2f> > >& estimatedTargetModelsWithIdentity = pTracker->getAgentEstimations();
			
#ifdef RESULTS_ENABLED
			if (agentId == 1)
			{
				if (estimatedTargetModelsWithIdentity.first.size() > 0)
				{
					results << "   <frame number=\"" << resultsIteration++ << "\">" << endl;
					results << "      <objectlist>" << endl;
				}
			}
#endif
			
#ifdef VISUALIZE_TRACKLETS
			googleScene = googleFrame.clone();
			
			/// Tracklets
			for (map<int,pair<pair<ObjectSensorReading::Observation,PTracking::Point2f>,pair<string,int> > >::const_iterator it = estimatedTargetModelsWithIdentity.first.begin();
																															 it != estimatedTargetModelsWithIdentity.first.end(); ++it)
			{
				cv::Point2f p;
				
				p.x = it->second.first.first.observation.getCartesian().x / resolution;
				p.y = it->second.first.first.observation.getCartesian().y / resolution;
				
				map<int,pair<int,pair<int,int> > >::iterator colorTrack = colorMap.find(it->first);
				
				const map<int,vector<PTracking::Point2f> >::iterator& h = targetHistory.find(it->first);
				
				if (h == targetHistory.end())
				{
					vector<PTracking::Point2f> history;
					
					history.push_back(PTracking::Point2f(p.x,p.y));
					
					targetHistory.insert(make_pair(it->first,history));
				}
				else
				{
					h->second.push_back(PTracking::Point2f(p.x,p.y));
					
					for (vector<PTracking::Point2f>::const_iterator it2 = h->second.begin(); it2 != h->second.end(); ++it2)
					{
						rectangle(googleScene,cvPoint(it2->x - 1, it2->y - 1),cvPoint(it2->x + 1, it2->y + 1),
								  cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second), 3);
						
						double imageX, imageY;
						
						imageX = (Hinv.at<double>(0,0) * it2->x + Hinv.at<double>(0,1) * it2->y + Hinv.at<double>(0,2)) / (Hinv.at<double>(2,0) * it2->x + Hinv.at<double>(2,1) * it2->y + Hinv.at<double>(2,2));
						imageY = (Hinv.at<double>(1,0) * it2->x + Hinv.at<double>(1,1) * it2->y + Hinv.at<double>(1,2)) / (Hinv.at<double>(2,0) * it2->x + Hinv.at<double>(2,1) * it2->y + Hinv.at<double>(2,2));
						
						rectangle(sceneMerged,cvPoint(imageX - 1, imageY - 1),cvPoint(imageX + 1, imageY + 1),
								  cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second), 3);
					}
				}
			}
			
			/// Google View
			if (agentId == 1)
			{
				for (map<int,pair<pair<ObjectSensorReading::Observation,PTracking::Point2f>,pair<string,int> > >::const_iterator it = estimatedTargetModelsWithIdentity.first.begin();
																																 it != estimatedTargetModelsWithIdentity.first.end(); ++it)
				{
					int fontFace = FONT_HERSHEY_SIMPLEX;
					double fontScale;
					int thickness;
					
					cv::Point2f p;
					
					p.x = it->second.first.first.observation.getCartesian().x / resolution;
					p.y = it->second.first.first.observation.getCartesian().y / resolution;
					
					map<int,pair<int,pair<int,int> > >::iterator colorTrack = colorMap.find(it->first);
					
					fontScale = 0.5;
					thickness = 3;
					
					Point textOrg;
					
					if (it->first < 9)
					{
						textOrg.x = p.x - 6;
						textOrg.y = p.y  - 11;
					}
					else
					{
						textOrg.x = p.x - 11;
						textOrg.y = p.y  - 11;
					}
					
					stringstream text;
					
					text << it->first;
					
					putText(googleScene, text.str(), textOrg, fontFace, fontScale, Scalar::all(0), thickness,8);
					
					fontScale = 0.5;
					thickness = 2;
					Point textOrg2;
					
					if (it->first < 9)
					{
						textOrg2.x = p.x - 4;
						textOrg2.y = p.y  - 12;
					}
					else
					{
						textOrg2.x = p.x - 9;
						textOrg2.y = p.y  - 12;
					}
					
					putText(googleScene,text.str(),textOrg2,fontFace,fontScale,Scalar::all(255),thickness,8);
					
					circle(googleScene,Point(p.x, p.y),6,cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),2);
				}
			}
			
			/// Re-projection of merged information on all views.
			for (map<int,pair<pair<ObjectSensorReading::Observation,PTracking::Point2f>,pair<string,int> > >::const_iterator it = estimatedTargetModelsWithIdentity.first.begin();
																															 it != estimatedTargetModelsWithIdentity.first.end(); ++it)
			{
				cv::Point2f p;
				
				p.x = it->second.first.first.observation.getCartesian().x / resolution;
				p.y = it->second.first.first.observation.getCartesian().y / resolution;
				
				double imageX, imageY;
				
				imageX = (Hinv.at<double>(0,0) * p.x + Hinv.at<double>(0,1) * p.y + Hinv.at<double>(0,2)) / (Hinv.at<double>(2,0) * p.x + Hinv.at<double>(2,1) * p.y + Hinv.at<double>(2,2));
				imageY = (Hinv.at<double>(1,0) * p.x + Hinv.at<double>(1,1) * p.y + Hinv.at<double>(1,2)) / (Hinv.at<double>(2,0) * p.x + Hinv.at<double>(2,1) * p.y + Hinv.at<double>(2,2));
				
				width = it->second.first.first.model.width;
				height = it->second.first.first.model.height;
				
				map<int,pair<int,pair<int,int> > >::iterator colorTrack = colorMap.find(it->first);
				
				rectangle(sceneMerged,cvPoint(imageX - (width / 2), imageY - height),cvPoint(imageX + (width / 2), imageY),
						  cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second), 3);
				
				stringstream text;
				
				text << it->first;
				
				int fontFace = FONT_HERSHEY_SIMPLEX;
				double fontScale = 1;
				int thickness = 3;
				Point textOrg(imageX - (width / 2), imageY - height - 7);
				putText(sceneMerged, text.str(), textOrg, fontFace, fontScale, Scalar::all(0), thickness,8);
				
				fontScale = 1;
				thickness = 2;
				Point textOrg2(imageX - (width / 2) + 2, imageY - height - 9);
				putText(sceneMerged, text.str(), textOrg2, fontFace, fontScale, Scalar::all(255), thickness,8);
			}
#endif
			
			if (!opticalTracker)
			{
				/// Plotting tracking data on the PlanarView.
				for (map<int,pair<pair<ObjectSensorReading::Observation,PTracking::Point2f>,pair<string,int> > >::const_iterator it = estimatedTargetModelsWithIdentity.first.begin();
																																 it != estimatedTargetModelsWithIdentity.first.end(); ++it)
				{
					cv::Point2f p;
					PTracking::Point2f a;
					Point textOrg, textOrg2;
					stringstream text;
					double angle, fontScale;
					int arrowMagnitude = 10, fontFace = FONT_HERSHEY_SIMPLEX, thickness;
					
					p.x = it->second.first.first.observation.getCartesian().x / resolution;
					p.y = it->second.first.first.observation.getCartesian().y / resolution;
					
					text << it->first;
					
					fontScale = 0.5;
					thickness = 3;
					
					if (it->first < 9)
					{
						textOrg.x = p.x - 6;
						textOrg.y = p.y	- 11;
					}
					else
					{
						textOrg.x = p.x - 11;
						textOrg.y = p.y	- 11;
					}
					
					putText(planarView,text.str(),textOrg,fontFace,fontScale,Scalar::all(0),thickness,8);
					
					fontScale = 0.5;
					thickness = 2;
					
					if (it->first < 9)
					{
						textOrg2.x = p.x - 4;
						textOrg2.y = p.y - 12;
					}
					else
					{
						textOrg2.x = p.x - 9;
						textOrg2.y = p.y - 12;
					}
					
					map<int,pair<int,pair<int,int> > >::iterator colorTrack = colorMap.find(it->first);
					
					putText(planarView,text.str(),textOrg2,fontFace,fontScale,Scalar::all(255),thickness,8);
					circle(planarView,Point(p.x,p.y),6,cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),2);
					
					a.x = it->second.first.first.observation.getCartesian().x;
					a.y = it->second.first.first.observation.getCartesian().y;
					
					PTracking::PointWithVelocity b = Utils::estimatedPosition(a,it->second.first.first.model.averagedVelocity,(fps / 1000) * 7);
					
					a.x /= resolution;
					a.y /= resolution;
					
					b.pose.x /= resolution;
					b.pose.y /= resolution;
					
					line(planarView,Point(a.x,a.y),Point(b.pose.x,b.pose.y),cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),3);
					
					angle = atan2(a.y - b.pose.y,a.x - b.pose.x);
					
					/// Compute the coordinates of the first segment.
					a.x = b.pose.x + (arrowMagnitude * cos(angle + M_PI / 4));
					a.y = b.pose.y + (arrowMagnitude * sin(angle + M_PI / 4));
					
					/// Draw the first segment.
					line(planarView,Point(a.x,a.y),Point(b.pose.x,b.pose.y),cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),3);
					
					/// Compute the coordinates of the second segment.
					a.x = b.pose.x + (arrowMagnitude * cos(angle - M_PI / 4));
					a.y = b.pose.y + (arrowMagnitude * sin(angle - M_PI / 4));
					
					/// Draw the second segment.
					line(planarView,Point(a.x,a.y),Point(b.pose.x,b.pose.y),cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),3);
				}
			}
			
			/// Projection of the estimations on the view.
			for (map<int,pair<ObjectSensorReading::Observation,PTracking::Point2f> >::const_iterator it = estimatedTargetModelsWithIdentity.second.begin();
																									 it != estimatedTargetModelsWithIdentity.second.end(); ++it)
			{
				cv::Point2f p;
				
				if (opticalTracker) resolution = 1;
				
				p.x = it->second.first.observation.getCartesian().x / resolution;
				p.y = it->second.first.observation.getCartesian().y / resolution;
				
				double imageX, imageY;
				
				if (opticalTracker)
				{
					imageX = p.x;
					imageY = p.y;
				}
				else
				{
					imageX = (Hinv.at<double>(0,0) * p.x + Hinv.at<double>(0,1) * p.y + Hinv.at<double>(0,2)) / (Hinv.at<double>(2,0) * p.x + Hinv.at<double>(2,1) * p.y + Hinv.at<double>(2,2));
					imageY = (Hinv.at<double>(1,0) * p.x + Hinv.at<double>(1,1) * p.y + Hinv.at<double>(1,2)) / (Hinv.at<double>(2,0) * p.x + Hinv.at<double>(2,1) * p.y + Hinv.at<double>(2,2));
				}
				
				width = it->second.first.model.width;
				height = it->second.first.model.height;
				
				map<int,pair<int,pair<int,int> > >::iterator colorTrack = colorMap.find(it->first);
				
				rectangle(scene,cvPoint(imageX - (width / 2), imageY - height),cvPoint(imageX + (width / 2), imageY),
						  cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second), 3);
				
				stringstream text;
				
				text << it->first;
				
				int fontFace = FONT_HERSHEY_SIMPLEX;
				double fontScale = 1;
				int thickness = 3;
				Point textOrg(imageX - (width / 2), imageY - height - 7);
				putText(scene, text.str(), textOrg, fontFace, fontScale, Scalar::all(0), thickness,8);
				
				fontScale = 1;
				thickness = 2;
				Point textOrg2(imageX - (width / 2) + 2, imageY - height - 9);
				putText(scene, text.str(), textOrg2, fontFace, fontScale, Scalar::all(255), thickness,8);
			}
			
#ifdef RESULTS_ENABLED
			for (map<int,pair<pair<ObjectSensorReading::Observation,PTracking::Point2f>,pair<string,int> > >::const_iterator it = estimatedTargetModelsWithIdentity.first.begin();
																															 it != estimatedTargetModelsWithIdentity.first.end(); ++it)
			{
				if (agentId == 1)
				{
					cv::Point2f p, h;
					
					if (opticalTracker) resolution = 1;
					
					p.x = it->second.first.first.observation.getCartesian().x / resolution;
					p.y = it->second.first.first.observation.getCartesian().y / resolution;
					
					h.x = it->second.first.first.head.x / resolution;
					h.y = it->second.first.first.head.y / resolution;
					
					double imageX, imageY, headImageX, headImageY;
					
					if (opticalTracker)
					{
						imageX = p.x;
						imageY = p.y;
						
						headImageX = h.x;
						headImageY = h.y;
					}
					else
					{
						imageX = (Hinv.at<double>(0,0) * p.x + Hinv.at<double>(0,1) * p.y + Hinv.at<double>(0,2)) / (Hinv.at<double>(2,0) * p.x + Hinv.at<double>(2,1) * p.y + Hinv.at<double>(2,2));
						imageY = (Hinv.at<double>(1,0) * p.x + Hinv.at<double>(1,1) * p.y + Hinv.at<double>(1,2)) / (Hinv.at<double>(2,0) * p.x + Hinv.at<double>(2,1) * p.y + Hinv.at<double>(2,2));
						
						headImageX = (Hinv.at<double>(0,0) * h.x + Hinv.at<double>(0,1) * h.y + Hinv.at<double>(0,2)) / (Hinv.at<double>(2,0) * h.x + Hinv.at<double>(2,1) * h.y + Hinv.at<double>(2,2));
						headImageY = (Hinv.at<double>(1,0) * h.x + Hinv.at<double>(1,1) * h.y + Hinv.at<double>(1,2)) / (Hinv.at<double>(2,0) * h.x + Hinv.at<double>(2,1) * h.y + Hinv.at<double>(2,2));
					}
					
					width = it->second.first.first.model.width;
					height = it->second.first.first.model.height;
					
					results << "         <object id=\"" << it->first << "\">" << endl;
					results << "            <box h=\"" << height << "\" w=\"" << width << "\" xc=\"" << imageX << "\""
							<< " yc=\"" << (imageY - (height / 2)) << "\""
							<< " hxc=\"" << headImageX << "\""
							<< " hyc=\"" << (headImageY + (height / 2)) << "\""
							<< " b=\"" << imageX << "\""
							<< "/>" << endl;
					results << "         </object>" << endl;
				}
			}
			
			if (agentId == 1)
			{
				if (estimatedTargetModelsWithIdentity.first.size() > 0)
				{
					results << "      </objectlist>" << endl;
					results << "   </frame>" << endl;
				}
			}
#endif
			
#ifdef VISUALIZE_TRACKLETS
			stringstream s2;
			
			s2 << s.str() << " (Tracklets)";
			
			imshow(s2.str(),sceneMerged);
			
			if (agentId == 1) imshow(dataset + string(" Planar View (Tracklets)"),googleScene);
#endif
			
			imshow(s.str(),scene);
			imshow(dataset + string(" Planar View"),planarView);
		}
		
		size_t index = fn.find_last_of("/");
		
		if (index == string::npos) index = fn.find_last_of("\\");
		
		size_t index2 = fn.find_last_of(".");
		string frameNumberString = fn.substr(index + 1,index2 - index - 1);
		
		rectangle(frame,Point(10,2),Point(100,20),Scalar(255,255,255),-1);
		putText(frame,frameNumberString.c_str(),Point(15,15),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0));
		
		//imshow("Foreground Mask",fgMask);
		//imshow("Background Model",bgImage);
		
		if (slow)
		{
			keyboard = waitKey(0);
			
			if ((char)keyboard == 'f') slow = false;
		}
		else
		{
			// SET-FPS
			keyboard = waitKey(1000.0 / (fps * 2));
			//keyboard = waitKey(0);
			if ((char) keyboard == 's') slow = true;	
		}
		
		int failsCounter;
		bool exit, ok;
		
		failsCounter = 0;
		exit = false;
		ok = false;
		
		while (!ok)
		{
			ostringstream oss;
			oss << (++frameNumber);
			string nextFrameNumberString = oss.str();
			
			stringstream nextFrameStream;
			
			nextFrameStream << setw(frameDigits.size()) << setfill('0') << nextFrameNumberString;
			
			nextFrameFilename = frameBaseName + nextFrameStream.str() + extension;
			
			frame = imread(nextFrameFilename);
			
			if (!frame.data)
			{
				++failsCounter;
				
				if (failsCounter == MAXIMUM_NUMBER_OF_MISSING_FRAMES)
				{
					exit = true;
					
					break;
				}
				
				ok = false;
			}
			else ok = true;
		}
		
		if (exit) break;
		
		fn.assign(nextFrameFilename);
		
#ifdef FRAME_RATE
		static float frameRate = 0.0;
		static int frameRateCounter = 0;
		
		if (pIMBS->isBackgroundCreated)
		{
			frameRate = ((frameRate * frameRateCounter) + (1000.0 / (Timestamp() - initialTimestamp).getMs())) / (frameRateCounter + 1);
			
			++frameRateCounter;
			
			ERR("FPS: " << frameRate << endl);
		}
#endif
	}
	
#ifdef RESULTS_ENABLED
	if (agentId == 1)
	{
		results << "</dataset>";
		
		results.close();
	}
#endif
	
	exit(EXIT_SUCCESS);
}

void searchSerialDevice()
{
	freenect_context* _ctx;
	freenect_init(&_ctx,0);
	
	struct freenect_device_attributes* list;
	struct freenect_device_attributes* item;
	
	freenect_list_device_attributes(_ctx,&list);
	
	int i = 0;
	
	INFO("Choose the device: " << std::endl);
	
	for (item = list; item != 0; item = item->next, i++)
	{
		WARN("\t "<< i + 1 << " - Found a camera with serial number: ");
		ERR(item->camera_serial << endl);
		
		serialList.push_back(std::string(item->camera_serial));
	}
	
	cerr << endl;
	
	bool isValid;
	
	isValid = false;
	
	do
	{
		INFO("Device: ");
		
		cin >> idx;
		
		if ((idx == 0) || (idx > (int) serialList.size()))
		{
			cerr << "\033[1A";
			cerr << "\033[K";
			
			cin.clear();
			cin.ignore(INT_MAX,'\n');
		}
		else isValid = true;
	}
	while (!isValid);
	
	--idx;
	
	freenect_free_device_attributes(list);
}

bool getData(const string& device, Kinect* kinect, VideoCapture& capture, Mat& image)
{
	if (device == "-kinect") return kinect->getData();
	else return capture.read(image);
}

int main(int argc, char* argv[])
{
	char calibfile[256];
	string dataset;
	bool calib = false;
	
	slow = false;
	
	help();
	
	if (argc < 3)
	{
		cerr <<"Incorrect input list" << endl;
		cerr <<"Exiting..." << endl;
		
		return EXIT_FAILURE;
	}
	
	if (strcmp(argv[1], "-vid") == 0)
	{
		ObservationManager* observationManager = 0;
		
		if (strcmp(argv[3],"-dataset") == 0) dataset = argv[4];
		
		if (argv[5] != 0)
		{
			if (strcmp(argv[5],"-agentId") == 0)
			{
				agentId = atoi(argv[6]);
				configure(string(getenv("PTracking_ROOT")) + string("/../config/parameters.cfg"),string(getenv("PTracking_ROOT")) + string("/../config/homography.cfg"),dataset);
			}
		}
		
		if (argc > 7)
		{
			if (strcmp(argv[7],"-savebg") == 0)
			{
				save_bg = true;
				save_bg_filename.assign(argv[8]);
			}
			else if (strcmp(argv[7],"-loadbg") == 0)
			{
				load_bg = true;
				load_bg_filename.assign(argv[8]);
			}
		}
		
		Etiseo::CameraModel *cam = 0;
		
		if (calib)
		{
			cam = new Etiseo::CameraModel;
			
			Etiseo::UtilXml::Init();
			
			ifstream is;
			is.open(calibfile);
			
			if (!is.is_open())
			{
				cerr << "Unable to open file " << calibfile << endl;
				cerr << "exiting..." << endl;
				
				return EXIT_FAILURE;
			}
			
			cam->fromXml(is);
		}
		
		observationManager = new ObservationManager(cam,H,resolution);
		
		int counter = 1;
		
		for (int i = 0; i < 256; i += 63)
		{
			for (int j = 0; j < 256; j += 63)
			{
				for (int k = 0; k < 256; k += 63, ++counter)
				{
					colorMap.insert(make_pair(counter,make_pair(i,make_pair(j,k))));
				}
			}
		}
		
		processVideo(argv[2],dataset,observationManager);
	}
	else if (strcmp(argv[1], "-img") == 0)
	{
		ObservationManager* observationManager = 0;
		
		if (strcmp(argv[3], "-fps") == 0)
		{
			fps = atof(argv[4]);
		}
		else fps = 25;
		
		if (strcmp(argv[5],"-dataset") == 0) dataset = argv[6];
		
		if (argv[7] != 0)
		{
			if (strcmp(argv[7],"-agentId") == 0)
			{
				agentId = atoi(argv[8]);
				configure(string(getenv("PTracking_ROOT")) + string("/../config/") + dataset + string("/parameters.cfg"),string(getenv("PTracking_ROOT")) + string("/../config/") + dataset + string("/homography.cfg"),dataset);
			}
		}
		
		if (argc > 10)
		{
			if (strcmp(argv[9],"-savebg") == 0)
			{
				save_bg = true;
				save_bg_filename.assign(argv[10]);
			}
			else if (strcmp(argv[9],"-loadbg") == 0)
			{
				load_bg = true;
				load_bg_filename.assign(argv[10]);
			}
		}
		
		Etiseo::CameraModel *cam = 0;
		
		if (calib)
		{
			cam = new Etiseo::CameraModel;
			
			Etiseo::UtilXml::Init();
			
			ifstream is;
			is.open(calibfile);
			
			if (!is.is_open())
			{
				cerr << "Unable to open file " << calibfile << endl;
				cerr << "exiting..." << endl;
				
				return EXIT_FAILURE;
			}
			
			cam->fromXml(is);
		}
		
		observationManager = new ObservationManager(cam,H,resolution);
		
		int counter = 1;
		
		for (int i = 0; i < 256; i += 63)
		{
			for (int j = 0; j < 256; j += 63)
			{
				for (int k = 0; k < 256; k += 63, ++counter)
				{
					colorMap.insert(make_pair(counter,make_pair(i,make_pair(j,k))));
				}
			}
		}
		
		processImages(argv[2],dataset,observationManager);
	}
	else if ((strcmp(argv[1], "-kinect") == 0) || (strcmp(argv[1], "-camera") == 0))
	{
		Kinect* kinectWriter = 0;
		VideoCapture capture;
		Mat image, scene, planarView, planarViewTemplate;
		stringstream counterStream;
		unsigned int counter;
		int offset;
		bool saveImages;
		
		if (strcmp(argv[2],"-fps") == 0) fps = atoi(argv[3]);
		
		if (strcmp(argv[4],"-agentId") == 0)
		{
			agentId = atoi(argv[5]);
			configure(string(getenv("PTracking_ROOT")) + string("/../config/Kinect/parameters.cfg"),string(getenv("PTracking_ROOT")) + string("/../config/Kinect/homography.cfg"),dataset,true);
		}
		
		saveImages = false;
		
		if ((argv[6] != 0) && (strcmp(argv[6],"-save") == 0)) saveImages = true;
		
		counter = 1;
		
		for (int i = 0; i < 256; i += 63)
		{
			for (int j = 0; j < 256; j += 63)
			{
				for (int k = 0; k < 256; k += 63, ++counter)
				{
					colorMap.insert(make_pair(counter,make_pair(i,make_pair(j,k))));
				}
			}
		}
		
		BackgroundSubtractorIMBS* pIMBS;
		
		pIMBS = new BackgroundSubtractorIMBS(fps);
		
		ObservationManager* observationManager = new ObservationManager(0,H,resolution);
		
		PTracker* pTracker;
		
		pTracker = new PTracker(agentId,string(getenv("PTracking_ROOT")) + string("/../config/Kinect/parameters.cfg"));
		
		if ((strcmp(argv[1], "-kinect") == 0))
		{
			searchSerialDevice();
			
			kinectWriter = new KinectWriter("",true,(char*) serialList[idx].c_str());
		}
		else
		{
			capture.open(1);
			
			if (!capture.isOpened())
			{
				ERR("Error when opening camera device. Exiting..." << endl);
				
				exit(0);
			}
		}
		
		offset = 24;
		
		planarViewTemplate = imread("../CameraView/InSpace/InSpace-PlanarView.png");
		
		namedWindow("InSpace Planar View");
		moveWindow("InSpace Planar View",640 - (planarViewTemplate.cols / 2),offset);
		
		namedWindow("Foreground Model");
		moveWindow("Foreground Model",0,planarViewTemplate.rows + 2 + (offset * 2));
		
		stringstream s;
		
		s << "PTracking - Camera " << agentId;
		
		namedWindow(s.str());
		moveWindow(s.str(),641,planarViewTemplate.rows + 2 + (offset * 2));
		
		counter = 0;
		
#ifdef RESULTS_ENABLED
		ofstream results;
		stringstream s2;
		
		struct stat temp;
		
		if (stat("../results",&temp) == -1)
		{
			mkdir("../results",0775);
		}
		
		s2 << "../results/PTracker-Kinect_" << agentId << ".xml";
		
		results.open(s2.str().c_str());
		
		try
		{
			ifstream usernameFile;
			string username;
			
			usernameFile.open("/etc/hostname");
			
			usernameFile >> username;
			
			/// Avoiding compilation warning.
			if (system((string("chown -R ") + username + string(".") + username + string(" ../results")).c_str()));
		}
		catch (...) {;}
		
		results << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << endl;
		results << "<dataset>" << endl;
		
		int resultsIteration;
		
		resultsIteration = 0;
#endif
		
		while (getData(argv[1],kinectWriter,capture,image))
		{
#ifdef FRAME_RATE
			Timestamp initialTimestamp;
#endif
			
			if (strcmp(argv[1], "-kinect") == 0) image = kinectWriter->frame;
			
			pIMBS->apply(image,fgMask);
			pIMBS->getBackgroundImage(bgImage);
			
			//imshow("Background Model",bgImage);
			
			const ObjectSensorReading& visualReading = observationManager->process(image,fgMask,opticalTracker);
			
			scene = image.clone();
			planarView = planarViewTemplate.clone();
			
			pTracker->exec(visualReading);
			
			const pair<map<int,pair<pair<ObjectSensorReading::Observation,PTracking::Point2f>,pair<string,int> > >,map<int,pair<ObjectSensorReading::Observation,PTracking::Point2f> > >& estimatedTargetModelsWithIdentity = pTracker->getAgentEstimations();
			
#ifdef RESULTS_ENABLED
			if (estimatedTargetModelsWithIdentity.first.size() > 0)
			{
				results << "   <frame number=\"" << resultsIteration++ << "\">" << endl;
				results << "      <objectlist>" << endl;
			}
#endif
			
			if (!opticalTracker)
			{
				/// Plotting tracking data on the InSpace-PlanarView.
				for (map<int,pair<pair<ObjectSensorReading::Observation,PTracking::Point2f>,pair<string,int> > >::const_iterator it = estimatedTargetModelsWithIdentity.first.begin();
																																 it != estimatedTargetModelsWithIdentity.first.end(); ++it)
				{
					cv::Point2f p;
					PTracking::Point2f a;
					Point textOrg, textOrg2;
					stringstream text;
					double angle, fontScale;
					int arrowMagnitude = 10, fontFace = FONT_HERSHEY_SIMPLEX, thickness;
					
					p.x = it->second.first.first.observation.getCartesian().x / resolution;
					p.y = it->second.first.first.observation.getCartesian().y / resolution;
					
					text << it->first;
					
					fontScale = 0.5;
					thickness = 3;
					
					if (it->first < 9)
					{
						textOrg.x = p.x - 6;
						textOrg.y = p.y	- 11;
					}
					else
					{
						textOrg.x = p.x - 11;
						textOrg.y = p.y	- 11;
					}
					
					putText(planarView,text.str(),textOrg,fontFace,fontScale,Scalar::all(0),thickness,8);
					
					fontScale = 0.5;
					thickness = 2;
					
					if (it->first < 9)
					{
						textOrg2.x = p.x - 4;
						textOrg2.y = p.y - 12;
					}
					else
					{
						textOrg2.x = p.x - 9;
						textOrg2.y = p.y - 12;
					}
					
					map<int,pair<int,pair<int,int> > >::iterator colorTrack = colorMap.find(it->first);
					
					putText(planarView,text.str(),textOrg2,fontFace,fontScale,Scalar::all(255),thickness,8);
					circle(planarView,Point(p.x,p.y),6,cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),2);
					
					a.x = it->second.first.first.observation.getCartesian().x;
					a.y = it->second.first.first.observation.getCartesian().y;
					
					PTracking::PointWithVelocity b = Utils::estimatedPosition(a,it->second.first.first.model.averagedVelocity,(fps / 1000) * 7);
					
					a.x /= resolution;
					a.y /= resolution;
					
					b.pose.x /= resolution;
					b.pose.y /= resolution;
					
					line(planarView,Point(a.x,a.y),Point(b.pose.x,b.pose.y),cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),3);
					
					angle = atan2(a.y - b.pose.y,a.x - b.pose.x);
					
					/// Compute the coordinates of the first segment.
					a.x = b.pose.x + (arrowMagnitude * cos(angle + M_PI / 4));
					a.y = b.pose.y + (arrowMagnitude * sin(angle + M_PI / 4));
					
					/// Draw the first segment.
					line(planarView,Point(a.x,a.y),Point(b.pose.x,b.pose.y),cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),3);
					
					/// Compute the coordinates of the second segment.
					a.x = b.pose.x + (arrowMagnitude * cos(angle - M_PI / 4));
					a.y = b.pose.y + (arrowMagnitude * sin(angle - M_PI / 4));
					
					/// Draw the second segment.
					line(planarView,Point(a.x,a.y),Point(b.pose.x,b.pose.y),cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),3);
				}
			}
			
			/// Plotting tracking data in the Kinect image.
			for (map<int,pair<ObjectSensorReading::Observation,PTracking::Point2f> >::const_iterator it = estimatedTargetModelsWithIdentity.second.begin();
																									 it != estimatedTargetModelsWithIdentity.second.end(); ++it)
			{
				cv::Point2f p;
				stringstream text;
				double fontScale;
				float imageX, imageY;
				int fontFace = FONT_HERSHEY_SIMPLEX, thickness;
				
				if (opticalTracker) resolution = 1;
				
				p.x = it->second.first.observation.getCartesian().x / resolution;
				p.y = it->second.first.observation.getCartesian().y / resolution;
				
				if (opticalTracker)
				{
					imageX = p.x;
					imageY = p.y;
				}
				else
				{
					imageX = (Hinv.at<double>(0,0) * p.x + Hinv.at<double>(0,1) * p.y + Hinv.at<double>(0,2)) / (Hinv.at<double>(2,0) * p.x + Hinv.at<double>(2,1) * p.y + Hinv.at<double>(2,2));
					imageY = (Hinv.at<double>(1,0) * p.x + Hinv.at<double>(1,1) * p.y + Hinv.at<double>(1,2)) / (Hinv.at<double>(2,0) * p.x + Hinv.at<double>(2,1) * p.y + Hinv.at<double>(2,2));
				}
				
				map<int,pair<int,pair<int,int> > >::iterator colorTrack = colorMap.find(it->first);
				
				rectangle(scene,cvPoint(imageX + it->second.first.model.boundingBox.first.x,imageY + it->second.first.model.boundingBox.first.y),
						  cvPoint(imageX + it->second.first.model.boundingBox.second.x,imageY + it->second.first.model.boundingBox.second.y),
						  cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),3);
				
				text << it->first;
				
				fontScale = 1;
				thickness = 3;
				
				Point textOrg(imageX + it->second.first.model.boundingBox.first.x,imageY + it->second.first.model.boundingBox.first.y - 7);
				putText(scene,text.str(),textOrg,fontFace,fontScale,Scalar::all(0),thickness,8);
				
				thickness = 2;
				
				Point textOrg2(imageX + it->second.first.model.boundingBox.first.x + 2,imageY + it->second.first.model.boundingBox.first.y - 9);
				putText(scene,text.str(),textOrg2,fontFace,fontScale,Scalar::all(255),thickness,8);
			}
			
#ifdef RESULTS_ENABLED
			for (map<int,pair<pair<ObjectSensorReading::Observation,PTracking::Point2f>,pair<string,int> > >::const_iterator it = estimatedTargetModelsWithIdentity.first.begin();
																															 it != estimatedTargetModelsWithIdentity.first.end(); ++it)
			{
				results << "         <object id=\"" << it->first << "\">" << endl;
				results << "            <box x=\"" << it->second.first.first.observation.getCartesian().x << "\" y=\"" << it->second.first.first.observation.getCartesian().y
						<< "\" vx=\"" <<  it->second.first.first.model.velocity.x << "\" vy=\"" << it->second.first.first.model.velocity.y << "\"/>" << endl;
				results << "         </object>" << endl;
			}
			
			if (estimatedTargetModelsWithIdentity.first.size() > 0)
			{
				results << "      </objectlist>" << endl;
				results << "   </frame>" << endl;
			}
#endif
			
			imshow(s.str(),scene);
			imshow("InSpace Planar View",planarView);
			
			if (saveImages)
			{
				counterStream.clear();
				counterStream.str("");
				
				counterStream << setw(5) << setfill('0') << counter;
				
				imwrite(string("rgb_") + counterStream.str() + string(".png"),image);
				//imwrite(string("depth_") + counterStream.str() + string(".png"),kinectWriter->depthMat16bit);
			}
			
			if (((char) waitKey(1000.0 / (fps * 2))) == 27) break;
			
#ifdef FRAME_RATE
			static float frameRate = 0.0;
			static int frameRateCounter = 0;
			
			if (pIMBS->isBackgroundCreated)
			{
				frameRate = ((frameRate * frameRateCounter) + (1000.0 / (Timestamp() - initialTimestamp).getMs())) / (frameRateCounter + 1);
				
				++frameRateCounter;
				
				ERR("FPS: " << frameRate << endl);
			}
#endif
			
			++counter;
		}
		
#ifdef RESULTS_ENABLED
		results << "</dataset>";
		
		results.close();
#endif
	}
	else
	{
		cerr <<"Please, check the input parameters." << endl;
		cerr <<"Exiting..." << endl;
		
		return EXIT_FAILURE;
	}
	
	destroyAllWindows();
	
	return EXIT_SUCCESS;
}
