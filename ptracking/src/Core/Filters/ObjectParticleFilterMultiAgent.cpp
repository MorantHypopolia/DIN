#include "ObjectParticleFilterMultiAgent.h"
#include "../Clusterizer/KClusterizer/KClusterizer.h"
#include "../Clusterizer/QTClusterizer/QTClusterizer.h"
#include "../SensorMaps/BasicSensorMap.h"
#include "../Sensors/BasicSensor.h"
#include "../../Utils/Timestamp.h"
#include <Manfield/utils/debugutils.h>
#include <Manfield/configfile/configfile.h>
#include <fstream>

using namespace std;
using GMapping::ConfigFile;
using GMapping::SensorReading;
using manfield::PrototypeFactory;
using manfield::SensorModel;

namespace PTracking
{
ObjectParticleFilterMultiAgent::ObjectParticleFilterMultiAgent(const string& type) : ParticleFilter(type), maxIdentityNumber(0) {;}
	
	ObjectParticleFilterMultiAgent::~ObjectParticleFilterMultiAgent() {;}
	
	PoseParticle ObjectParticleFilterMultiAgent::adjustWeight(const PoseParticle& p, unsigned long particlesTimestamp, unsigned long currentTimestamp, Utils::DecreaseModelFactor model, float factor) const
	{
		PoseParticle g;
		
		if (model == Utils::Linear)
		{
			g.pose = p.pose;
			
			/// The timestamp is in milliseconds that is why we divide by 1000.0.
			g.weight -= (factor * (((float) (currentTimestamp - particlesTimestamp)) / 1000.0));
			
			if (g.weight < 0.0) g.weight = 0.0;
		}
		
		return g;
	}
	
	void ObjectParticleFilterMultiAgent::configure(const string& filename)
	{
		ConfigFile fCfg;
		string key, section, temp;
		float worldXMin = 0, worldXMax = 0, worldYMin = 0, worldYMax = 0;
		bool opticalTracker;
		
		if (!fCfg.read(filename))
		{
			ERR("Error reading file '" << filename << "' for filter configuration. Exiting..." << endl);
			
			exit(-1);
		}
		
		ParticleFilter::configure(filename);
		
		try
		{
			section = "parameters";
			
			key = "bestParticles";
			bestParticlesNumber = fCfg.value(section,key);
			
			key = "closenessThreshold";
			closenessThreshold = fCfg.value(section,key);
			
			key = "opticalTracker";
			opticalTracker = fCfg.value(section,key);
			
			key = "timeToWaitBeforeDeleting";
			timeToWaitBeforeDeleting = fCfg.value(section,key);
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
		string filterName = getname();
		
		SensorModel* sm = 0;
		
		try
		{
			section = "sensormodel";
			key = "type";
			
			sm = PrototypeFactory<SensorModel>::forName(fCfg.value(section,key));
			
			if (sm != 0)
			{
				m_sensorModel = sm;
			}
			else
			{
				ERR("Missing prototype for sensorModel " << string(fCfg.value(section,key)) << ". Exiting..." << endl);
				
				exit(-1);
			}
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
		m_sensorModel->configure(filename,new BasicSensor(filterName));
		
		BasicSensorMap* basicSensorMap = new BasicSensorMap();
		
		/// Reading sensor locations.
		section = "location";
		
		try
		{
			key = "worldXMin";
			temp = string(fCfg.value(section,key));
			
			if (temp == "-inf") worldXMin = FLT_MIN;
			else worldXMin = atof(temp.c_str());
			
			key = "worldXMax";
			temp = string(fCfg.value(section,key));
			
			if (temp == "inf") worldXMax = FLT_MAX;
			else worldXMax = atof(temp.c_str());
			
			key = "worldYMin";
			temp = string(fCfg.value(section,key));
			
			if (temp == "-inf") worldYMin = FLT_MIN;
			else worldYMin = atof(temp.c_str());
			
			key = "worldYMax";
			temp = string(fCfg.value(section,key));
			
			if (temp == "inf") worldYMax = FLT_MAX;
			else worldYMax = atof(temp.c_str());
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
		basicSensorMap->setworldMin(Point2f(worldXMin,worldYMin));
		basicSensorMap->setworldMax(Point2f(worldXMax,worldYMax));
		
		/// Type of clustering.
		section = "clustering";
		
		try
		{
			key = "algorithm";
			clusteringAlgorithm = string(fCfg.value(section,key));
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
		if (strcasecmp(clusteringAlgorithm.c_str(),"KClusterizer") == 0) clusterizer = new KClusterizer(1);
		else if (strcasecmp(clusteringAlgorithm.c_str(),"QTClusterizer") == 0) clusterizer = new QTClusterizer();
		
		Point2f p;
		ifstream ifs;
		string ifsFilename = filename, sensorName;
		char buf[256];
		
		ifs.open(ifsFilename.c_str());
		
		while (ifs.good())
		{
			if (ifs.eof()) break;
			
			ifs.getline(buf,256);
			
			if (string(buf).compare(0,filterName.length(),filterName) == 0)
			{
				istringstream iss(buf);
				
				iss >> sensorName >> p.x >> p.y;
				
				basicSensorMap->insertSensor(p);
			}
		}
		
		ifs.close();
		
		m_sensorModel->setSensorMap(basicSensorMap);
		
		closenessThreshold = min(0.8f,2 * closenessThreshold);
		
		ERR("######################################################" << endl);
		DEBUG("Distributed particle filter parameters:" << endl);
		INFO("\tCloseness object threshold (in " << (opticalTracker ? "pixels" : "meters") << "): " << closenessThreshold << endl);
		DEBUG("\tTime to wait before deleting: " << timeToWaitBeforeDeleting << " ms" << endl);
		ERR("######################################################" << endl << endl);
	}
	
	bool ObjectParticleFilterMultiAgent::isSameDirection(const pair<int,pair<ObjectSensorReading::Observation,Point2f> >& e1, const pair<int,pair<ObjectSensorReading::Observation,Point2f> >& e2) const
	{
		return (Utils::calculateEstimationDirection(e1.second.first.model.velocity) == Utils::calculateEstimationDirection(e2.second.first.model.velocity));
	}
	
	void ObjectParticleFilterMultiAgent::observe(const vector<ObjectSensorReadingMultiAgent>& readings)
	{
		PoseParticleVector fusedParticles;
		
		currentTimestamp = Timestamp().getMsFromMidnight();
		
		for (vector<ObjectSensorReadingMultiAgent>::const_iterator it = readings.begin(); it != readings.end(); ++it)
		{
			const map<int,pair<ObjectSensorReading::Observation,Point2f> >& estimationsWithtModels = it->getEstimationsWithModels();
			
			for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it2 = estimationsWithtModels.begin(); it2 != estimationsWithtModels.end(); ++it2)
			{
				const PoseParticleVector& part = Utils::samplingParticles(it2->second.first.observation.getCartesian(),it2->second.first.sigma,bestParticlesNumber);
				
				for (PoseParticleVector::const_iterator it3 = part.begin(); it3 != part.end(); it3++)
				{
					fusedParticles.push_back(adjustWeight(*it3,it->getEstimationsTimestamp(),currentTimestamp,Utils::Linear,0.15));
				}
			}
		}
		
		m_params.m_particles = fusedParticles;
		
		for (PoseParticleVector::iterator i = m_params.m_particles.begin(); i != m_params.m_particles.end(); i++)
		{
			if (i->weight > m_params.m_maxParticle.weight)
			{
				m_params.m_maxParticle = *i;
			}
		}
		
		updateTargetIdentity(readings);
		
		//clusterizer->clusterize(m_params.m_particles,closenessThreshold);
		//clusters = clusterizer->getClusters();
		
		resample();
	}
	
	void ObjectParticleFilterMultiAgent::resetWeight()
	{
		int i, size;
		
		i = 0;
		size = m_params.m_particles.size();
		
		/// Partial Loop Unrolling to better use pipeling.
		for (; i < size - 3; i += 4)
		{
			m_params.m_particles.at(i).weight = 1.0;
			m_params.m_particles.at(i + 1).weight = 1.0;
			m_params.m_particles.at(i + 2).weight = 1.0;
			m_params.m_particles.at(i + 3).weight = 1.0;
		}
		
		for (; i < size; ++i)
		{
			m_params.m_particles.at(i).weight = 1.0;
		}
	}
	
	void ObjectParticleFilterMultiAgent::updateTargetIdentity(const vector<ObjectSensorReadingMultiAgent>& readings)
	{
		vector<pair<ObjectSensorReading::Observation,Point2f> > estimationsToBeFused;
		float allSigmaX, allSigmaY, distance, globalEstimationX, globalEstimationY, globalEstimationHeadX, globalEstimationHeadY, minDistance, sigmaNormalizationRatioX, sigmaNormalizationRatioY;
		int currentIndex;
		
		currentIndex = -1;
		
		/// Analyzing each agent.
		for (vector<ObjectSensorReadingMultiAgent>::const_iterator it = readings.begin(); it != readings.end(); ++it)
		{
			const map<int,pair<ObjectSensorReading::Observation,Point2f> >& estimations = it->getEstimationsWithModels();
			
			/// Analyzing all the estimations performed by the agent.
			for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it2 = estimations.begin(); it2 != estimations.end(); ++it2)
			{
				estimationsToBeFused.clear();
				estimationsToBeFused.push_back(it2->second);
				
				vector<ObjectSensorReadingMultiAgent>::const_iterator it3;
				
				it3 = it;
				advance(it3,1);
				
				/// Analyzing all the other agents.
				for (; it3 != readings.end(); ++it3)
				{
					map<int,pair<ObjectSensorReading::Observation,Point2f> >& estimationsOtherAgent = *const_cast<map<int,pair<ObjectSensorReading::Observation,Point2f> >*>(&it3->getEstimationsWithModels());
					
					/// Analyzing all the estimations performed by the other agent.
					for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::iterator it4 = estimationsOtherAgent.begin(); it4 != estimationsOtherAgent.end(); )
					{
						//if (isSameDirection(*it2,*it4))
						{
							/// Keeping estimations having the same direction and that are sufficiently close each other.
							if (Utils::isTargetNear(it2->second.first.observation.getCartesian(),it4->second.first.observation.getCartesian(),closenessThreshold))
							{
								estimationsToBeFused.push_back(it4->second);
								estimationsOtherAgent.erase(it4++);
							}
							else ++it4;
						}
						//else ++it4;
					}
				}
				
				allSigmaX = 0.0;
				allSigmaY = 0.0;
				
				for (vector<pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it4 = estimationsToBeFused.begin(); it4 != estimationsToBeFused.end(); ++it4)
				{
					allSigmaX += it4->second.x;
					allSigmaY += it4->second.y;
				}
				
				/// Fusing estimations.
				if (estimationsToBeFused.size() > 1)
				{
					globalEstimationX = 0.0;
					globalEstimationY = 0.0;
					
					globalEstimationHeadX = 0.0;
					globalEstimationHeadY = 0.0;
					
					sigmaNormalizationRatioX = 0.0;
					sigmaNormalizationRatioY = 0.0;
					
					for (vector<pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it5 = estimationsToBeFused.begin(); it5 != estimationsToBeFused.end(); ++it5)
					{
						sigmaNormalizationRatioX += (1.0 - (it5->second.x / allSigmaX));
						sigmaNormalizationRatioY += (1.0 - (it5->second.y / allSigmaY));
					}
					
					sigmaNormalizationRatioX = 1.0 / sigmaNormalizationRatioX;
					sigmaNormalizationRatioY = 1.0 / sigmaNormalizationRatioY;
					
					for (vector<pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it5 = estimationsToBeFused.begin(); it5 != estimationsToBeFused.end(); ++it5)
					{
						globalEstimationX += (it5->first.observation.getCartesian().x * ((1.0 - (it5->second.x / allSigmaX)) * sigmaNormalizationRatioX));
						globalEstimationY += (it5->first.observation.getCartesian().y * ((1.0 - (it5->second.y / allSigmaY)) * sigmaNormalizationRatioY));
						
						globalEstimationHeadX += (it5->first.head.x * ((1.0 - (it5->second.x / allSigmaX)) * sigmaNormalizationRatioX));
						globalEstimationHeadY += (it5->first.head.y * ((1.0 - (it5->second.y / allSigmaY)) * sigmaNormalizationRatioY));
					}
				}
				else
				{
					globalEstimationX = estimationsToBeFused.begin()->first.observation.getCartesian().x;
					globalEstimationY = estimationsToBeFused.begin()->first.observation.getCartesian().y;
					
					globalEstimationHeadX = estimationsToBeFused.begin()->first.head.x;
					globalEstimationHeadY = estimationsToBeFused.begin()->first.head.y;
				}
				
				ObjectSensorReading::Observation globalEstimation;
				
				globalEstimation.observation.rho = sqrt((globalEstimationX * globalEstimationX) + (globalEstimationY * globalEstimationY));
				globalEstimation.observation.theta = atan2(globalEstimationY,globalEstimationX);
				globalEstimation.head.x = globalEstimationHeadX;
				globalEstimation.head.y = globalEstimationHeadY;
				globalEstimation.sigma = Point2f(allSigmaX / estimationsToBeFused.size(), allSigmaY / estimationsToBeFused.size());
				globalEstimation.model = it2->second.first.model;
				
				minDistance = FLT_MAX;
				
				/// Finding the current index of such an estimation, if any.
				for (map<int,pair<pair<ObjectSensorReading::Observation,Point2f>,pair<string,int> > >::const_iterator it3 = estimatedTargetModelsWithIdentityMultiAgent.begin(); it3 != estimatedTargetModelsWithIdentityMultiAgent.end(); ++it3)
				{
					const Point2f& e = it3->second.first.first.observation.getCartesian();
					
					distance = sqrt(((globalEstimationX - e.x) * (globalEstimationX - e.x)) + ((globalEstimationY - e.y) * (globalEstimationY - e.y)));
					
					if (distance < minDistance)
					{
						minDistance = distance;
						currentIndex = it3->first;
					}
				}
				
				if (minDistance <= closenessThreshold)
				{
					const map<int,pair<pair<ObjectSensorReading::Observation,Point2f>,pair<string,int> > >::iterator& estimation = estimatedTargetModelsWithIdentityMultiAgent.find(currentIndex);
					
					estimation->second.first.first = globalEstimation;
					estimation->second.first.second = globalEstimation.sigma;
					
					const map<int,Timestamp>::iterator& estimationTime = estimationsMultiAgentUpdateTime.find(currentIndex);
					
					/// Updating the timestamp of the estimation.
					estimationTime->second.setToNow();
				}
				else
				{
					int targetIdentity;
					
					/// Checking if	the local identity of the object can be used. At this moment, I do not care about which agent provides the identity of the object, that is why I am using the first one.
					const map<int,pair<pair<ObjectSensorReading::Observation,Point2f>,pair<string,int> > >::iterator& estimation = estimatedTargetModelsWithIdentityMultiAgent.find(it2->first);
					
					if (estimation == estimatedTargetModelsWithIdentityMultiAgent.end()) targetIdentity = it2->first;
					else targetIdentity = ++maxIdentityNumber;
					
					estimatedTargetModelsWithIdentityMultiAgent.insert(make_pair(targetIdentity,make_pair(make_pair(globalEstimation,globalEstimation.sigma),it->getAgent())));
					estimationsMultiAgentUpdateTime.insert(make_pair(targetIdentity,Timestamp()));
				}
			}
		}
		
		/// Removing all the estimations that have not been associated for a period equals to timeToWaitBeforeDeleting.
		for (map<int,pair<pair<ObjectSensorReading::Observation,Point2f>,pair<string,int> > >::iterator it = estimatedTargetModelsWithIdentityMultiAgent.begin(); it != estimatedTargetModelsWithIdentityMultiAgent.end(); )
		{
			const map<int,Timestamp>::iterator& estimationTime = estimationsMultiAgentUpdateTime.find(it->first);
			
			if (((long) currentTimestamp - (long) estimationTime->second.getMsFromMidnight()) > (long) timeToWaitBeforeDeleting)
			{
				estimatedTargetModelsWithIdentityMultiAgent.erase(it++);
				estimationsMultiAgentUpdateTime.erase(estimationTime);
			}
			else ++it;
		}
	}
	
	SENSORFILTER_FACTORY(ObjectParticleFilterMultiAgent)
}
