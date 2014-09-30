#include "ObjectSensorReading.h"
#include <Utils/Utils.h>

using namespace std;

namespace PTracking
{
	ObjectSensorReading::ObjectSensorReading() {;}
	
	ObjectSensorReading::~ObjectSensorReading() {;}
	
	void ObjectSensorReading::setObservations(const vector<Observation>& obs)
	{
		observations = obs;
	}
	
	void ObjectSensorReading::setObservations(Observation targetPoints[], int currentTargetIndex, int lastCurrentTargetIndex, int lastNTargetPerception, const Point2f& worldX, const Point2f& worldY)
	{
		float x, y;
		int i;
		
		observations.clear();
		
		i = lastCurrentTargetIndex;
		
		while (i != currentTargetIndex)
		{
			x = targetPoints[i].observation.getCartesian().x;
			y = targetPoints[i].observation.getCartesian().y;
			
			/// The x value represents the minimum world limit while the y value represents the maximum one.
			if (((x >= worldX.x) && (x <= worldX.y)) && ((y >= worldY.x) && (y <= worldY.y)))
			{
				const Point2of& result = Utils::convertRelative2Global(Point2of(x,y,0.0),observationsAgentPose);
				
				targetPoints[i].observation.rho = result.mod();
				targetPoints[i].observation.theta = atan2(result.y,result.x);
				
				observations.push_back(targetPoints[i]);
			}
			
			++i;
			
			/// To avoid an infinity loop.
			if (i == currentTargetIndex) break;
			
			/// Because targetPoints is a circular buffer.
			if (i == lastNTargetPerception) i = 0;
		}
	}
	
	void ObjectSensorReading::setObservationsAgentPose(const Point2of& pose)
	{
		observationsAgentPose = pose;
	}
	
	void ObjectSensorReading::setSensor(const BasicSensor& s)
	{
		sensor = s;
	}
}
