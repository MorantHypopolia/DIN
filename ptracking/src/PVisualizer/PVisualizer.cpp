#include "PVisualizer.h"
#include "../PTracker/PTracker.h"
#include <opencv2/highgui/highgui.hpp>
#include <libxml/xmlreader.h>
#include <signal.h>

/// Macros used by the xml reader function.
#define XML_TAG_DATASET			BAD_CAST"dataset"
#define XML_TAG_FRAME			BAD_CAST"frame"
#define XML_TAG_OBJECT_LIST		BAD_CAST"objectlist"
#define XML_TAG_OBJECT			BAD_CAST"object"
#define XML_TAG_BOX				BAD_CAST"box"

#define XML_TAG_OBJECT_ID		BAD_CAST"id"
#define XML_TAG_OBJECT_HEIGHT	BAD_CAST"h"
#define XML_TAG_OBJECT_WIDTH	BAD_CAST"w"
#define XML_TAG_OBJECT_XC		BAD_CAST"xc"
#define XML_TAG_OBJECT_YC		BAD_CAST"yc"

using namespace std;
using namespace cv;
using namespace PTracking;

PVisualizer::PVisualizer()
{
	signal(SIGINT,PVisualizer::interruptCallback);
	
	int counter = 1;
	
	for (int i = 0; i < 256; i += 63)
	{
		for (int j = 0; j < 256; j += 63)
		{
			for (int k = 0; k < 256; k += 128, ++counter)
			{
				colorMap.insert(make_pair(counter,make_pair(i,make_pair(j,k))));
			}
		}
	}
}

PVisualizer::~PVisualizer() {;}

void PVisualizer::exec(const string& imagePath, const string& resultsXmlFile)
{
	map<int,vector<pair<PTracking::Point2f,PTracking::Point2f> > > targetHistory;
	cv::Point2f p, q;
	float angle;
	int i, j, arrowMagnitude;
	
	Mat image = imread(imagePath);
	
	if (image.empty())
	{ 
		ERR("Error loading image: " << imagePath << ". Exiting..." << endl);
		
		exit(-1);
	}
	
	arrowMagnitude = 20;
	
	const vector<pair<vector<int>,ObjectSensorReading> >& visualReadings = readEstimationFile(resultsXmlFile);
	
	for (vector<pair<vector<int>,ObjectSensorReading> >::const_iterator it = visualReadings.begin(); it != visualReadings.end(); ++it)
	{
		const vector<ObjectSensorReading::Observation>& observations = it->second.getObservations();
		const vector<int>& identities = it->first;
		
		i = 0;
		
		for (vector<ObjectSensorReading::Observation>::const_iterator it2 = observations.begin(); it2 != observations.end(); ++it2, ++i)
		{
			map<int,pair<int,pair<int,int> > >::iterator colorTrack = colorMap.find(identities.at(i));
			
			const map<int,vector<pair<PTracking::Point2f,PTracking::Point2f> > >::iterator& h = targetHistory.find(identities.at(i));
			
			if (h == targetHistory.end())
			{
				vector<pair<PTracking::Point2f,PTracking::Point2f> > history;
				
				history.push_back(make_pair(PTracking::Point2f(it2->observation.getCartesian().x,it2->observation.getCartesian().y),
											PTracking::Point2f(it2->model.width,it2->model.height)));
				
				targetHistory.insert(make_pair(identities.at(i),history));
				
				rectangle(image,cvPoint(it2->observation.getCartesian().x - 10,it2->observation.getCartesian().y - 10),
						  cvPoint(it2->observation.getCartesian().x + 10,it2->observation.getCartesian().y + 10),
						  cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),3);
			}
			else
			{
				h->second.push_back(make_pair(PTracking::Point2f(it2->observation.getCartesian().x,it2->observation.getCartesian().y),
											  PTracking::Point2f(it2->model.width,it2->model.height)));
				
				j = 0;
				
				for (vector<pair<PTracking::Point2f,PTracking::Point2f> >::const_iterator it3 = h->second.begin(); it3 != h->second.end(); ++it3, ++j)
				{
					rectangle(image,cvPoint(it3->first.x - 1,it3->first.y - 1),cvPoint(it3->first.x + 1,it3->first.y + 1),
							  cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),3);
					
					if (j > 149)
					{
						vector<pair<PTracking::Point2f,PTracking::Point2f> >::const_iterator prev = it3;
						
						advance(prev,-1);
						
						p.x = it3->first.x;
						p.y = it3->first.y;
						
						q.x = prev->first.x;
						q.y = prev->first.y;						
						
						angle = atan2(q.y - p.y,q.x - p.x);
						
						/// Compute the coordinates of the first segment.
						p.x = q.x + (arrowMagnitude * cos(angle + M_PI / 4));
						p.y = q.y + (arrowMagnitude * sin(angle + M_PI / 4));
						
						/// Draw the first segment.
						line(image,p,q,cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),3);
						
						/// Compute the coordinates of the second segment.
						p.x = q.x + (arrowMagnitude * cos(angle - M_PI / 4));
						p.y = q.y + (arrowMagnitude * sin(angle - M_PI / 4));
						
						/// Draw the second segment.
						line(image,p,q,cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),3);
						
						j = 0;
					}
				}
			}
		}
		
		imshow("Trajectories - press 'q' to exit, press 's' to save the image (when the plot is completed))",image);
		
		char key = waitKey(10);
		
		if (key == 'q') exit(0);
	}
	
	char key;
	
	key = waitKey(0);
	
	if (key == 's')
	{
		INFO("Saving images...");
		
		imwrite("../results/trajectories.png",image);
		
		INFO("done." << endl);
	}
}

void PVisualizer::interruptCallback(int)
{
	ERR(endl << "*********************************************************************" << endl);
	ERR("Caught Ctrl+C. Exiting..." << endl);
	ERR("*********************************************************************" << endl);
	
	exit(0);
}

vector<pair<vector<int>,ObjectSensorReading> > PVisualizer::readEstimationFile(const string& estimationFile)
{
	vector<pair<vector<int>,ObjectSensorReading> > visualReadings;
	xmlDocPtr file;
	xmlNodePtr frame;
	
	file = xmlReadFile(estimationFile.c_str(),"UTF-8",XML_PARSE_RECOVER);
	
	if (file == 0)
	{
		ERR("Error reading file '" << estimationFile << "'. Exiting..." << endl);
		
		exit(-1);
	}
	
	frame = xmlDocGetRootElement(file);
	
	if (xmlStrcmp(frame->name,XML_TAG_DATASET))
	{
		ERR("File '" << estimationFile << "' in a wrong format. Exiting..." << endl);
		
		exit(-1);
	}
	
	frame = frame->children;		
	
	while (frame != 0)
	{
		if (!xmlStrcmp(frame->name,XML_TAG_FRAME))
		{
			xmlNodePtr objectList;
			
			objectList = frame->children;
			
			while (objectList != 0)
			{
				if (!xmlStrcmp(objectList->name,XML_TAG_OBJECT_LIST))
				{
					vector<ObjectSensorReading::Observation> observations;
					vector<int> identities;
					ObjectSensorReading visualReading;
					
					xmlNodePtr object = objectList->children;
					
					while (object != 0)
					{
						if (!xmlStrcmp(object->name,XML_TAG_OBJECT))
						{
							xmlChar* temp;
							float x, y;
							
							temp = xmlGetProp(object,XML_TAG_OBJECT_ID);
							
							xmlNodePtr box = object->children;
							
							while (box != 0)
							{
								if (!xmlStrcmp(box->name,XML_TAG_BOX))
								{
									ObjectSensorReading::Observation obs;
									
									identities.push_back(atoi((char*) temp));
									
									temp = xmlGetProp(box,XML_TAG_OBJECT_HEIGHT);
									obs.model.height = atof((char*) temp);
									xmlFree(temp);
									
									temp = xmlGetProp(box,XML_TAG_OBJECT_WIDTH);
									obs.model.width = atof((char*) temp);
									xmlFree(temp);
									
									temp = xmlGetProp(box,XML_TAG_OBJECT_XC);
									x = atof((char*) temp);
									xmlFree(temp);
									
									temp = xmlGetProp(box,XML_TAG_OBJECT_YC);
									y = atof((char*) temp);
									xmlFree(temp);
									
									obs.observation.rho = sqrt((x * x) + (y * y));
									obs.observation.theta = atan2(y,x);
									
									observations.push_back(obs);
								}
								
								box = box->next;
							}
						}
						
						object = object->next;
					}
					
					visualReading.setObservations(observations);
					
					visualReadings.push_back(make_pair(identities,visualReading));
				}
				
				objectList = objectList->next;
			}
		}
		
		frame = frame->next;
	}
	
	return visualReadings;
}
