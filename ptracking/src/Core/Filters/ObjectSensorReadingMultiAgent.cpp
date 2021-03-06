#include "ObjectSensorReadingMultiAgent.h"

using namespace std;

namespace PTracking
{
	ObjectSensorReadingMultiAgent::ObjectSensorReadingMultiAgent() {;}
	
	ObjectSensorReadingMultiAgent::~ObjectSensorReadingMultiAgent() {;}
	
	void ObjectSensorReadingMultiAgent::setAgent(const string& address, int port)
	{
		observationMultiAgent.address = address;
		observationMultiAgent.port = port;
	}
	
	void ObjectSensorReadingMultiAgent::setEstimationsTimestamp(unsigned long timestamp)
	{
		observationMultiAgent.timestamp = timestamp;
	}
	
	void ObjectSensorReadingMultiAgent::setEstimationsWithModels(const map<int,pair<ObjectSensorReading::Observation,Point2f> >& estimationsWithModels)
	{
		observationMultiAgent.estimationsWithModels = estimationsWithModels;
	}
	
	void ObjectSensorReadingMultiAgent::setSensor(const BasicSensor& s)
	{
		sensor = s;
	}
}
