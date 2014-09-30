#include "ObjectParticleFilter.h"
#include "../Clusterizer/KClusterizer/KClusterizer.h"
#include "../Clusterizer/QTClusterizer/QTClusterizer.h"
#include "../SensorModels/BasicSensorModel.h"
#include "../Sensors/BasicSensor.h"
#include <Utils/Utils.h>
#include <Manfield/configfile/configfile.h>
#include <math.h>
#include <algorithm>
#include <iterator>

/// Uncomment to enable debug prints.
//#define DEBUG_MODE

using namespace std;
using GMapping::ConfigFile;
using manfield::PrototypeFactory;
using manfield::SensorModel;

namespace PTracking
{
	ObjectParticleFilter::ObjectParticleFilter(const string& type) : ParticleFilter(type), clusterizer(0), maxIdentityNumber(0) {;}
	
	ObjectParticleFilter::~ObjectParticleFilter()
	{
		if (clusterizer != 0) delete clusterizer;
	}
	
	void ObjectParticleFilter::addParticlesInInterestingPoints(ObjectSensorReading& readings)
	{
		float distance, minDistance;
		int clusterIndex, counterIndex;
		bool associated;
		
		numberOfObservationAssociatedAndPromoted = 0;
		
		observeNeeded = (timeToWaitBeforePromoting == 0) ? true : false;
		
		vector<ObjectSensorReading::Observation>& obs = readings.getObservations();
		
		for (vector<ObjectSensorReading::Observation>::iterator it = obs.begin(); it != obs.end(); )
		{
#ifdef DEBUG_MODE
			DEBUG("Analizying observation: [" << it->observation.getCartesian().x << "," << it->observation.getCartesian().y << "]" << endl);
#endif
			
			minDistance = FLT_MAX;
			clusterIndex = -1;
			counterIndex = 0;
			associated = false;
			
			for (vector<pair<PoseParticleVector,Point2of> >::iterator it2 = clusters.begin(); it2 != clusters.end(); ++it2, ++counterIndex)
			{
				distance = sqrt(((it->observation.getCartesian().x - it2->second.x) * (it->observation.getCartesian().x - it2->second.x)) +
								((it->observation.getCartesian().y - it2->second.y) * (it->observation.getCartesian().y - it2->second.y)));
				
				/// A cluster does not have to be already associated to an observation.
				if ((distance < minDistance) && (observationsMapping.find(counterIndex) == observationsMapping.end()))
				{
					minDistance = distance;
					clusterIndex = counterIndex;
				}
			}
			
			if (minDistance <= closenessThreshold)
			{
				associated = true;
				observeNeeded = true;
				++numberOfObservationAssociatedAndPromoted;
				
				observationsMapping.insert(make_pair(clusterIndex,make_pair(*it,clusters.at(clusterIndex).second)));
				
#ifdef DEBUG_MODE
				WARN("Associated observation [" << it->observation.getCartesian().x << "," << it->observation.getCartesian().y << "] to cluster [" << clusters.at(clusterIndex).second.x << "," << clusters.at(clusterIndex).second.y << "]" << endl);
#endif
			}
			
			if (!associated)
			{
				for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::iterator it2 = estimatedTargetModelsWithIdentity.begin(); it2 != estimatedTargetModelsWithIdentity.end(); ++it2)
				{
					if (Utils::isTargetNear(it->observation.getCartesian(),it2->second.first.observation.getCartesian(),closenessThreshold))
					{
						associated = true;
						observeNeeded = true;
						++numberOfObservationAssociatedAndPromoted;
						
#ifdef DEBUG_MODE
						INFO("Associated observation [" << it->observation.getCartesian().x << "," << it->observation.getCartesian().y << "] to estimation ["
														<< it2->second.first.observation.getCartesian().x << "," << it2->second.first.observation.getCartesian().y << "]" << endl);
#endif
						
						break;
					}
				}
			}
			
			if (!associated)
			{
				bool exists;
				
				exists = false;
				
				/// I am looking for a pending observation that can be matched with the current one.
				for (vector<pair<Point2of,pair<Timestamp,Timestamp> > >::iterator it2 = pendingObservations.begin(); it2 != pendingObservations.end(); )
				{
					if (Utils::isTargetNear(it->observation.getCartesian(),it2->first,closenessThreshold))
					{
						Timestamp temp;
						
						if (((temp - it2->second.first).getMs() < 500.0) && ((temp - it2->second.second).getMs() >= timeToWaitBeforePromoting))
						{
#ifdef DEBUG_MODE
							ERR("Promoting observation: [" << it2->first.x << "," << it2->first.y << "]" << endl);
#endif
							
							associated = true;
							observeNeeded = true;
							it2 = pendingObservations.erase(it2);
						}
						else
						{
							exists = true;
							
							it2->first.x = it->observation.getCartesian().x;
							it2->first.y = it->observation.getCartesian().y;
							it2->second.first = temp;
							
#ifdef DEBUG_MODE
							DEBUG("Updating time of pending observation: [" << it2->first.x << "," << it2->first.y << "]" << endl);
#endif
							
							++it2;
						}
						
						break;
					}
					else ++it2;
				}
				
				if (associated || (timeToWaitBeforePromoting == 0))
				{
					PoseParticleVector particlesNewPromotedObservation;
					PointWithVelocity pointWithVelocity;
					Point2of temp(it->observation.getCartesian());
					
					pointWithVelocity.pose.x = temp.x;
					pointWithVelocity.pose.y = temp.y;
					pointWithVelocity.pose.theta = temp.theta;
					
#ifdef DEBUG_MODE
					INFO("Promoted observation: [" << pointWithVelocity.pose.x << "," << pointWithVelocity.pose.y << "]" << endl);
#endif
					
					if ((numberOfObservationAssociatedAndPromoted == 0) && (clusters.size() == 0)) m_params.m_particles.clear();
					
					fill_n(back_inserter(particlesNewPromotedObservation),getparticleNumber() * (((numberOfObservationAssociatedAndPromoted == 0) && (clusters.size() == 0)) ? 1.0 : 0.05),
										 PoseParticle(pointWithVelocity,1.0));
					
					for (PoseParticleVector::iterator it2 = particlesNewPromotedObservation.begin(); it2 != particlesNewPromotedObservation.end(); ++it2)
					{
						/// Adding Gaussian noise.
						Point2of noise(Utils::sampleGaussianSigma(m_params.sr0),Utils::sampleGaussianSigma(m_params.sr0),Utils::sampleGaussianSigma(m_params.st0));
						
						it2->pose.pose.x += noise.x;
						it2->pose.pose.y += noise.y;
						it2->pose.pose.theta = Utils::angNormPiSig(it2->pose.pose.theta + noise.theta);
					}
					
					m_params.m_particles.insert(m_params.m_particles.end(),particlesNewPromotedObservation.begin(),particlesNewPromotedObservation.end());
					
					++numberOfObservationAssociatedAndPromoted;
				}
				else if (!exists)
				{
					pendingObservations.push_back(make_pair(it->observation.getCartesian(),make_pair(Timestamp(),Timestamp())));
					
#ifdef DEBUG_MODE
					WARN("Adding pending observation: [" << it->observation.getCartesian().x << "," << it->observation.getCartesian().y << "]" << endl);
#endif
				}
			}
			
			if (associated || (timeToWaitBeforePromoting == 0)) ++it;
			else it = obs.erase(it);
		}
		
		/// Removing false pending observations.
		for (vector<pair<Point2of,pair<Timestamp,Timestamp> > >::iterator it = pendingObservations.begin(); it != pendingObservations.end(); )
		{
			if ((Timestamp() - it->second.first).getMs() > timeToWaitBeforeDeleting)
			{
				it = pendingObservations.erase(it);
				
#ifdef DEBUG_MODE
				INFO("Removing pending observation: [" << it->first.x << "," << it->first.y << "]" << endl);
#endif
			}
			else ++it;
		}
		
#ifdef DEBUG_MODE
		DEBUG("Number of observations associated and promoted: " << numberOfObservationAssociatedAndPromoted << endl);
#endif
	}
	
	void ObjectParticleFilter::applyGroupTracking()
	{
		vector<pair<int,pair<ObjectSensorReading::Observation,Point2f> > > overlappingEstimatedTargetModels;
		map<int,pair<ObjectSensorReading::Observation,Point2f> > estimatedTargetModelsWithIdentityCopy;
		stringstream s;
		float boundingBoxOverlappingFactor, x1, x2, y1, y2, width1, width2, height1, height2;
		
		estimatedTargetModelsWithIdentityCopy = estimatedTargetModelsWithIdentity;
		
		groupEstimatedTargetModelsWithIdentity.clear();
		
		boundingBoxOverlappingFactor = 0.8;
		
		/// Checking whether multiple objects have to be merged in a group.
		for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::iterator it = estimatedTargetModelsWithIdentityCopy.begin(); it != estimatedTargetModelsWithIdentityCopy.end(); )
		{
			map<int,pair<ObjectSensorReading::Observation,Point2f> >::iterator it2;
			
			it2 = it;
			advance(it2,1);
			
			overlappingEstimatedTargetModels.clear();
			s.clear();
			s.str("");
			
			for (; it2 != estimatedTargetModelsWithIdentityCopy.end(); )
			{
				x1 = it->second.first.observation.getCartesian().x - (it->second.first.model.width / 2.0) + ((1.0 - boundingBoxOverlappingFactor) * (it->second.first.model.width / 2.0));
				y1 = it->second.first.observation.getCartesian().y - it->second.first.model.height + ((1.0 - boundingBoxOverlappingFactor) * (it->second.first.model.height / 2.0));
				x2 = it2->second.first.observation.getCartesian().x - (it2->second.first.model.width / 2.0) + ((1.0 - boundingBoxOverlappingFactor) * (it2->second.first.model.width / 2.0));
				y2 = it2->second.first.observation.getCartesian().y - it2->second.first.model.height + ((1.0 - boundingBoxOverlappingFactor) * (it2->second.first.model.height / 2.0));
				
				width1 = it->second.first.model.width * boundingBoxOverlappingFactor;
				width2 = it2->second.first.model.width * boundingBoxOverlappingFactor;
				
				height1 = it->second.first.model.height * boundingBoxOverlappingFactor;
				height2 = it2->second.first.model.height * boundingBoxOverlappingFactor;
				
				if ((x1 + width1 < x2) || (x2 + width2 < x1) || (y1 + height1 < y2) || (y2 + height2 < y1)) ++it2;
				else
				{
					overlappingEstimatedTargetModels.push_back(*it2);
					
					estimatedTargetModelsWithIdentityCopy.erase(it2++);
				}
			}
			
			if (!overlappingEstimatedTargetModels.empty())
			{
				Point2f groupSigma;
				float barycenter, heightMax, heightMin, widthMax, widthMin;
				
				overlappingEstimatedTargetModels.push_back(*it);
				
				widthMax = FLT_MIN;
				widthMin = FLT_MAX;
				heightMax = FLT_MIN;
				heightMin = FLT_MAX;
				
				for (vector<pair<int,pair<ObjectSensorReading::Observation,Point2f> > >::const_iterator it2 = overlappingEstimatedTargetModels.begin(); it2 != overlappingEstimatedTargetModels.end(); ++it2)
				{
					s << it2->first << " ";
					
					x1 = it2->second.first.observation.getCartesian().x;
					y1 = it2->second.first.observation.getCartesian().y;
					
					if ((x1 - ((it2->second.first.model.width / 2.0) * boundingBoxOverlappingFactor)) < widthMin) widthMin = x1 - ((it2->second.first.model.width / 2.0) * boundingBoxOverlappingFactor);
					if ((x1 + ((it2->second.first.model.width / 2.0) * boundingBoxOverlappingFactor)) > widthMax) widthMax = x1 + ((it2->second.first.model.width / 2.0) * boundingBoxOverlappingFactor);
					
					if ((y1 - (it2->second.first.model.height * boundingBoxOverlappingFactor)) < heightMin) heightMin = y1 - (it2->second.first.model.height * boundingBoxOverlappingFactor);
					if ((y1 * boundingBoxOverlappingFactor) > heightMax) heightMax = y1 * boundingBoxOverlappingFactor;
					
					groupSigma.x = it2->second.second.x;
					groupSigma.y = it2->second.second.y;
				}
				
				barycenter = widthMin + ((widthMax - widthMin) / 2.0);
				
				ObjectSensorReading::Observation groupEstimation;
				
				groupEstimation.observation = PolarPoint(atan2(heightMax,barycenter),Point2f(barycenter,heightMax).mod());
				groupEstimation.model.barycenter = barycenter;
				groupEstimation.model.height = heightMax - heightMin;
				groupEstimation.model.width = widthMax - widthMin;
				
				groupEstimatedTargetModelsWithIdentity.push_back(make_pair(s.str().substr(0,s.str().size() - 1),make_pair(groupEstimation,groupSigma / overlappingEstimatedTargetModels.size())));
				
				estimatedTargetModelsWithIdentityCopy.erase(it++);
				
#ifdef DEBUG_MODE
				ERR("BUILDING GROUP:" << endl);
				
				for (vector<pair<int,pair<ObjectSensorReading::Observation,Point2f> > >::const_iterator it2 = overlappingEstimatedTargetModels.begin(); it2 != overlappingEstimatedTargetModels.end(); ++it2)
				{
					INFO("\t[" << it2->first << "] -> (" << it2->second.first.observation.getCartesian().x << "," << it2->second.first.observation.getCartesian().y << ")" << endl);
				}
#endif
			}
			else
			{
				s << it->first;
				
				groupEstimatedTargetModelsWithIdentity.push_back(make_pair(s.str(),it->second));
				
				++it;
			}
		}
	}
	
	pair<bool,float> ObjectParticleFilter::areModelsSimilar(const ObjectSensorReading::Model& observationModel, const ObjectSensorReading::Model& storedModel) const
	{
		float bhattacharyyaCoefficients[3] = { 0.0, 0.0, 0.0 };
		float bhattacharyyaDistance;
		
		for (int i = 0; i < ObjectSensorReading::Model::HISTOGRAM_VECTOR_LENGTH; ++i)
		{
			bhattacharyyaCoefficients[0] += sqrt(observationModel.histograms[0][i] * storedModel.histograms[0][i]);
			bhattacharyyaCoefficients[1] += sqrt(observationModel.histograms[1][i] * storedModel.histograms[1][i]);
			bhattacharyyaCoefficients[2] += sqrt(observationModel.histograms[2][i] * storedModel.histograms[2][i]);
		}
		
		bhattacharyyaDistance = 0.0;
		
		for (int i = 0; i < 3; ++i) bhattacharyyaDistance += ((3.0 - i) / 6.0) * sqrt(1 - bhattacharyyaCoefficients[i]);
		
		return make_pair(bhattacharyyaDistance < 0.35,bhattacharyyaDistance);
	}
	
	bool ObjectParticleFilter::checkFilterForReinitialization()
	{
		if ((Timestamp().getMsFromMidnight() - lastObserveTimestamp) < 5000.0) return false;
		
		lastObserveTimestamp = currentTimestamp;
		
#ifdef DEBUG_MODE
		ERR("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl);
		ERR("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl);
		ERR("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl);
		ERR("\t\tRe-initialization performed!!!" << endl);
		ERR("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl);
		ERR("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl);
		ERR("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl);
#endif
		
		/// Re-initialization needed since enough time without estimations has passed.
		clusters.clear();
		estimatedTargetModelsWithIdentity.clear();
		estimatedTargetMovements.clear();
		estimationsUpdateTime.clear();
		groupEstimatedTargetModelsWithIdentity.clear();
		observationsMapping.clear();
		pendingObservations.clear();
		
		delete clusterizer;
		
		if (strcasecmp(clusteringAlgorithm.c_str(),"KClusterizer") == 0) clusterizer = new KClusterizer(1);
		else if (strcasecmp(clusteringAlgorithm.c_str(),"QTClusterizer") == 0) clusterizer = new QTClusterizer();
		
		return true;
	}
	
	void ObjectParticleFilter::configure(const string& filename)
	{
		ConfigFile fCfg;
		string key, section, temp;
		float worldXMin = 0, worldXMax = 0, worldYMin = 0, worldYMax = 0;
		
		if (!fCfg.read(filename))
		{
			ERR("Error reading file '" << filename << "' for filter configuration. Exiting..." << endl);
			
			exit(-1);
		}
		
		ParticleFilter::configure(filename);
		
		try
		{
			section = "parameters";
			
			key = "averagedVelocityWindow";
			averagedVelocityWindow = fCfg.value(section,key);
			
			key = "closenessThreshold";
			closenessThreshold = fCfg.value(section,key);
			
			key = "opticalTracker";
			opticalTracker = fCfg.value(section,key);
			
			key = "timeToWaitBeforeDeleting";
			timeToWaitBeforeDeleting = fCfg.value(section,key);
			
			key = "timeToWaitBeforePromoting";
			timeToWaitBeforePromoting = fCfg.value(section,key);
			
			key = "velocityStabilizationFactor";
			velocityStabilizationFactor = fCfg.value(section,key);
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
			
			key = "modelLinearVelocity";
			modelLinearVelocity =  fCfg.value(section,key);
		}
		catch (...)
		{
			ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
			
			exit(-1);
		}
		
		m_sensorModel->configure(filename,new BasicSensor(filterName));
		
		BasicSensorMap* basicSensorMap = new BasicSensorMap();
		
		try
		{
			/// Reading sensor locations.
			section = "location";
			
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
		
		try
		{
			/// Type of clustering.
			section = "clustering";
			
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
		
		ERR(endl << "******************************************************" << endl);
		DEBUG("Particle filter parameters:" << endl);
		
		INFO("\tMap size: [" << (worldXMax - worldXMin) << "," << (worldYMax - worldYMin) << "] " << (opticalTracker ? "pixels" : "meters") << endl);
		
		WARN("\tOptical tracking: ");
		ERR((opticalTracker ? "enabled" : "disabled") << endl);
		
		INFO("\tModel linear velocity: " << modelLinearVelocity << (opticalTracker ? " pixels/s" : " m/s") << endl);
		DEBUG("\tTime to wait before deleting: " << timeToWaitBeforeDeleting << " ms" << endl);
		WARN("\tTime to wait before promoting: " << timeToWaitBeforePromoting << " ms" << endl);
		INFO("\tCloseness object threshold (in " << (opticalTracker ? "pixels" : "meters") << "): " << closenessThreshold << endl);
		ERR("******************************************************" << endl << endl);
	}
	
	void ObjectParticleFilter::normalizeWeight()
	{
		float totalWeight;
		
		for (vector<pair<PoseParticleVector,Point2of> >::iterator it = clusters.begin(); it != clusters.end(); it++)
		{
			totalWeight = 0.0;
			
			/// Calculation of total weight of the particles of cluster.
			for (PoseParticleVector::iterator it2 = it->first.begin(); it2 != it->first.end(); it2++)
			{
				totalWeight += it2->weight;
			}
			
			/// Normalize weight of the particles of cluster.
			for (PoseParticleVector::iterator it2 = it->first.begin(); it2 != it->first.end(); it2++)
			{
				it2->weight /= totalWeight;
			}
		}
		
		m_params.m_maxParticle.weight = 0.0;
		
		for (PoseParticleVector::iterator it = m_params.m_particles.begin(); it != m_params.m_particles.end(); it++)
		{
			if (it->weight > m_params.m_maxParticle.weight)
			{
				m_params.m_maxParticle = *it;
			}
		}
	}
	
	void ObjectParticleFilter::observe(ObjectSensorReading& readings)
	{
#ifdef DEBUG_MODE
		ERR(endl << "********************************************************" << endl);
#endif
		
		static bool isReinitialized = true;
		
		observationsMapping.clear();
		
		/// Sort in decreasing order.
		sort(readings.getObservations().begin(),readings.getObservations().end(),Utils::compareObservation);
		
		if (!isReinitialized)
		{
			if (checkFilterForReinitialization()) isReinitialized = true;
		}
		
		addParticlesInInterestingPoints(readings);
		
#ifdef DEBUG_MODE
		ERR("Is observe needed?: ");
		INFO((observeNeeded ? "yes" : "no") << endl);
#endif
		
		if (!observeNeeded)
		{
			/// Removing all the estimations that have not been associated for a period equals to timeToWaitBeforeDeleting.
			for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::iterator it = estimatedTargetModelsWithIdentity.begin(); it != estimatedTargetModelsWithIdentity.end(); )
			{
				const map<int,Timestamp>::iterator& estimationTime = estimationsUpdateTime.find(it->first);
				
				if (((long) currentTimestamp - (long) estimationTime->second.getMsFromMidnight()) > (long) timeToWaitBeforeDeleting)
				{
#ifdef DEBUG_MODE
					INFO("Deleting old estimation (" << it->first << ") -> [" << it->second.first.observation.getCartesian().x << "," << it->second.first.observation.getCartesian().y << "], " <<
						 "time = " << ((long) currentTimestamp - (long) estimationTime->second.getMsFromMidnight()) << " ms" << endl);
#endif
					
					estimatedTargetModelsWithIdentity.erase(it++);
					estimationsUpdateTime.erase(estimationTime);
				}
				else
				{
#ifdef DEBUG_MODE
					WARN("Keeping old estimation (" << it->first << ") -> [" << it->second.first.observation.getCartesian().x << "," << it->second.first.observation.getCartesian().y << "]" << endl);
#endif
					
					if ((fabs(it->second.first.model.velocity.x) <= modelLinearVelocity) && (fabs(it->second.first.model.velocity.y) <= modelLinearVelocity))
					{
						const PointWithVelocity& newPose = Utils::estimatedPosition(it->second.first.observation.getCartesian(),it->second.first.model.velocity,dt);
						
						it->second.first.observation.rho = sqrt((newPose.pose.x * newPose.pose.x) + (newPose.pose.y * newPose.pose.y));
						it->second.first.observation.theta = atan2(newPose.pose.y,newPose.pose.x);
					}
					
					++it;
				}
			}
			
			return;
		}
		
		isReinitialized = false;
		
		lastObserveTimestamp = currentTimestamp;
		
		resetWeight();
		
		const BasicSensorModel* basicSensorModel = static_cast<BasicSensorModel*>(getSensorModel());
		
		PointIterator begin = m_params.m_particles.begin();
		PointIterator end = m_params.m_particles.end();
		
		basicSensorModel->likelihood(&readings,begin,end);
		
		/// Adapting target's number.
		if (numberOfObservationAssociatedAndPromoted > 0)
		{
			if (numberOfObservationAssociatedAndPromoted >= clusters.size())
			{
				clusterizer->setMaxClusterNumber(numberOfObservationAssociatedAndPromoted);
				
				lastTimeShouldBeDecreased.setToNow();
			}
			else
			{
				if ((Timestamp() - lastTimeShouldBeDecreased).getMs() >= timeToWaitBeforePromoting)
				{
					clusterizer->setMaxClusterNumber((clusterizer->getCurrentClusterNumber() > 1) ? (clusterizer->getCurrentClusterNumber() - 1) : 1);
				}
			}
		}
		
		clusterizer->clusterize(m_params.m_particles,(opticalTracker) ? 20 : 0.45);
		clusters = clusterizer->getClusters();
		
		/// Sort in decreasing order.
		sort(clusters.begin(),clusters.end(),Utils::comparePairPoint2of);
		
		updateTargetIdentity(readings);
		
		if (opticalTracker) applyGroupTracking();
		
		normalizeWeight();
		resample();
		
		/// Sort in decreasing order.
		sort(m_params.m_particles.begin(),m_params.m_particles.end(),Utils::comparePoseParticle);		
		
		m_params.m_particles.erase(m_params.m_particles.begin() + getparticleNumber(),m_params.m_particles.end());
		
#ifdef DEBUG_MODE
		int i;
		
		i = 1;
		
		for (vector<pair<PoseParticleVector,Point2of> >::iterator it = clusters.begin(); it != clusters.end(); ++it, ++i)
		{
			DEBUG("Cluster (" << i << ") -> [" << it->second.x << "," << it->second.y << "]" << endl);
		}
		
		ERR("########################################################" << endl);
		
		for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it = estimatedTargetModelsWithIdentity.begin(); it != estimatedTargetModelsWithIdentity.end(); ++it)
		{
			WARN("Estimation with identity (" << it->first << ") -> [" << it->second.first.observation.getCartesian().x << "," << it->second.first.observation.getCartesian().y << "], " <<
				 "velocity = (" << it->second.first.model.velocity.x << "," << it->second.first.model.velocity.y << "), " <<
				 "averaged velocity = (" << it->second.first.model.averagedVelocity.x << "," << it->second.first.model.averagedVelocity.y << ")" << endl);
		}
		
		ERR("********************************************************" << endl);
#endif
	}
	
	void ObjectParticleFilter::predict(const Point2of& newRobotPose, const Point2of& oldRobotPose, const Timestamp& initialTimestamp, const Timestamp& current)
	{
		Point2f velocity;
		float a, b, c, d, deltaX, deltaY, deltaTheta, deltaTheta2;
		unsigned int bestParticlesEachCluster;
		
		bestParticlesEachCluster = 0;
		currentTimestamp = current.getMsFromMidnight();
		
		if (estimatedTargetModelsWithIdentity.size() > 0)
		{
			bestParticlesEachCluster = getparticleNumber() / estimatedTargetModelsWithIdentity.size();
			
			m_params.m_particles.clear();
		}
		
		/// Because the timestamps are in milliseconds.
		dt = (static_cast<float>(currentTimestamp - initialTimestamp.getMsFromMidnight()) / 1000.0);
		
		for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::iterator it = estimatedTargetModelsWithIdentity.begin(); it != estimatedTargetModelsWithIdentity.end(); ++it)
		{
			const map<int,Point2f>::const_iterator& correspondence = estimatedTargetMovements.find(it->first);
			
			if (correspondence != estimatedTargetMovements.end())
			{
				velocity.x = correspondence->second.x / dt;
				velocity.y = correspondence->second.y / dt;
				
				if (velocity.x > modelLinearVelocity) velocity.x = modelLinearVelocity;
				else if (velocity.x < -modelLinearVelocity) velocity.x = -modelLinearVelocity;
				
				if (velocity.y > modelLinearVelocity) velocity.y = modelLinearVelocity;
				else if (velocity.y < -modelLinearVelocity) velocity.y = -modelLinearVelocity;
			}
			else
			{
				velocity.x = 0.0;
				velocity.y = 0.0;
			}
			
			const PointWithVelocity& newPose = Utils::estimatedPosition(it->second.first.observation.getCartesian(),velocity,dt);
			
			PoseParticleVector part;
			
			fill_n(back_inserter(part),bestParticlesEachCluster,PoseParticle(newPose,1.0));
			
			for (PoseParticleVector::iterator it2 = part.begin(); it2 != part.end(); ++it2)
			{
				/// Adding Gaussian noise.
				Point2of noise(Utils::sampleGaussianSigma(m_params.sr0),Utils::sampleGaussianSigma(m_params.sr0),Utils::sampleGaussianSigma(m_params.st0));
				
				it2->pose.pose.x += noise.x;
				it2->pose.pose.y += noise.y;
				it2->pose.pose.theta = Utils::angNormPiSig(it2->pose.pose.theta + noise.theta);
				
				m_params.m_particles.push_back(*it2);
			}
		}
		
		if (m_params.m_particles.size() < getparticleNumber())
		{
			unsigned int difference;
			
			difference = getparticleNumber() - m_params.m_particles.size();
			
			for (unsigned int j = 0; j < difference; j++) m_params.m_particles.push_back(m_params.m_particles.at(j));
		}
		
		/// I am considering the robot's movement too. So, the particles are moved coherently.
		deltaX = cos(-newRobotPose.theta) * (newRobotPose.x - oldRobotPose.x) - sin(-newRobotPose.theta) * (newRobotPose.y - oldRobotPose.y);
		deltaY = sin(-newRobotPose.theta) * (newRobotPose.x - oldRobotPose.x) + cos(-newRobotPose.theta) * (newRobotPose.y - oldRobotPose.y);
		deltaTheta = Utils::angNormPiSig(newRobotPose.theta - oldRobotPose.theta);
		deltaTheta2 = deltaTheta / 2;
		
		a = cos(oldRobotPose.theta + deltaTheta2);
		b = cos(M_PI / 2 + oldRobotPose.theta + deltaTheta2);
		
		c = sin(oldRobotPose.theta + deltaTheta2);
		d = sin(M_PI / 2 + oldRobotPose.theta + deltaTheta2);
		
		for (PoseParticleVector::iterator it = m_params.m_particles.begin(); it != m_params.m_particles.end(); ++it)
		{
			/// Runge-Kutta approximation.
			it->pose.pose.x += (deltaX * a) + (deltaY * b);
			it->pose.pose.y += (deltaX * c) + (deltaY * d);
			it->pose.pose.theta = Utils::angNormPiSig(it->pose.pose.theta + deltaTheta);
		}
		
		/// Adding some noise to the estimations because when no targets are detected this leads to increase their deviation standard.
		for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::iterator it = estimatedTargetModelsWithIdentity.begin(); it != estimatedTargetModelsWithIdentity.end(); ++it)
		{
			it->second.first.sigma.x += 0.01;
			it->second.first.sigma.y += 0.01;
			it->second.second = it->second.first.sigma;
		}
	}
	
	void ObjectParticleFilter::resample()
	{
		for (vector<pair<PoseParticleVector,Point2of> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
		{
			vector<int> indexes;
			
			indexes.reserve(2 * it->first.size());
			::resample(indexes,it->first);
			repeatIndexes(it->first,indexes,it->first);
		}
	}
	
	void ObjectParticleFilter::resetWeight()
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
	
	void ObjectParticleFilter::updateTargetIdentity(ObjectSensorReading& readings)
	{
		static map<int,list<pair<float,float> > > averagedVelocities;
		
		map<int,pair<ObjectSensorReading::Observation,Point2f> > estimatedTargetModelsWithIdentityPreviousIteration;
		vector<int> estimationsValid, deletingClusters, pendingClusters;
		Point2f associatedObservation;
		stringstream s;
		float distance, minDistance;
		int clusterIndex, i, index;
		
		estimatedTargetModelsWithIdentityPreviousIteration = estimatedTargetModelsWithIdentity;
		
		vector<ObjectSensorReading::Observation>& obs = readings.getObservations();
		
		i = 0;
		
		/// Removing clusters that do not have a close observation (due to particles' convergence).
		for (vector<pair<PoseParticleVector,Point2of> >::iterator it = clusters.begin(); it != clusters.end(); ++it, ++i)
		{
#ifdef DEBUG_MODE
			ERR("Analyzing cluster: [" << it->second.x << "," << it->second.y << "]" << endl);
#endif
			
			minDistance = FLT_MAX;
			
			for (vector<ObjectSensorReading::Observation>::iterator it2 = obs.begin(); it2 != obs.end(); ++it2)
			{
				const Point2f& o = it2->observation.getCartesian();
				
				distance = sqrt(((it->second.x - o.x) * (it->second.x - o.x)) + ((it->second.y - o.y) * (it->second.y - o.y)));
				
				if (distance < minDistance)
				{
					associatedObservation = o;
					minDistance = distance;
				}
			}
			
			/// Found an observation close to the cluster.
			if (minDistance < closenessThreshold)
			{
#ifdef DEBUG_MODE
				WARN("Cluster [" << it->second.x << "," << it->second.y << "] associated to observation (" << associatedObservation.x << "," << associatedObservation.y << ")" << endl);
#endif
			}
			else
			{
				minDistance = FLT_MAX;
				index = -1;
				
				for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::iterator it2 = estimatedTargetModelsWithIdentity.begin(); it2 != estimatedTargetModelsWithIdentity.end(); ++it2)
				{
					const Point2f& o = it2->second.first.observation.getCartesian();
					
					distance = sqrt(((it->second.x - o.x) * (it->second.x - o.x)) + ((it->second.y - o.y) * (it->second.y - o.y)));
					
					if (distance < minDistance)
					{
						minDistance = distance;
						index = it2->first;
					}
				}
				
				/// Even if we could not find an observation close to the cluster, we found an estimation close to it.
				if (minDistance < closenessThreshold)
				{
					const map<int,Timestamp>::iterator& estimationTime = estimationsUpdateTime.find(index);
					
#ifdef DEBUG_MODE
					INFO("Found estimation [" << index << "] close to the cluster: [" << it->second.x << "," << it->second.y << "]" << endl);
#endif
					
					if (estimationTime != estimationsUpdateTime.end())
					{
						/// Cluster have to be deleted.
						if ((currentTimestamp - estimationTime->second.getMsFromMidnight()) > timeToWaitBeforeDeleting)
						{
#ifdef DEBUG_MODE
							DEBUG("Deleting cluster: [" << it->second.x << "," << it->second.y << "], time = " << (currentTimestamp - estimationTime->second.getMsFromMidnight()) << " ms" << endl);
#endif
							
							deletingClusters.push_back(i);
						}
						else
						{
#ifdef DEBUG_MODE
							WARN("Keeping cluster: [" << it->second.x << "," << it->second.y << "]" << endl);
#endif
							
							/// We consider this estimation as valid since it is close to the pending cluster.
							estimationsValid.push_back(index);
							pendingClusters.push_back(i);
							
							pair<ObjectSensorReading::Observation,Point2f>& est = estimatedTargetModelsWithIdentity.at(index);
							
							if ((fabs(est.first.model.velocity.x) <= modelLinearVelocity) && (fabs(est.first.model.velocity.y) <= modelLinearVelocity))
							{
								const PointWithVelocity& newPose = Utils::estimatedPosition(it->second,est.first.model.velocity,dt);
								
								it->second.x = newPose.pose.x;
								it->second.y = newPose.pose.y;
								
								est.first.observation.rho = sqrt((newPose.pose.x * newPose.pose.x) + (newPose.pose.y * newPose.pose.y));
								est.first.observation.theta = atan2(newPose.pose.y,newPose.pose.x);
							}
						}
					}
					else
					{
#ifdef DEBUG_MODE
						INFO("Keeping cluster: [" << it->second.x << "," << it->second.y << "]" << endl);
#endif
						
						/// We consider this estimation as valid since it is close to the pending cluster.
						estimationsValid.push_back(index);
						pendingClusters.push_back(i);
						
						pair<ObjectSensorReading::Observation,Point2f>& est = estimatedTargetModelsWithIdentity.at(index);
						
						if ((fabs(est.first.model.velocity.x) <= modelLinearVelocity) && (fabs(est.first.model.velocity.y) <= modelLinearVelocity))
						{
							const PointWithVelocity& newPose = Utils::estimatedPosition(it->second,est.first.model.velocity,dt);
							
							it->second.x = newPose.pose.x;
							it->second.y = newPose.pose.y;
							
							est.first.observation.rho = sqrt((newPose.pose.x * newPose.pose.x) + (newPose.pose.y * newPose.pose.y));
							est.first.observation.theta = atan2(newPose.pose.y,newPose.pose.x);
						}
					}
				}
				else
				{
#ifdef DEBUG_MODE
					WARN("Deleting cluster without observations: [" << it->second.x << "," << it->second.y << "]" << endl);
#endif
					
					deletingClusters.push_back(i);
				}
			}
		}
		
		clusterIndex = 0;
		
		for (vector<pair<PoseParticleVector,Point2of> >::iterator it = clusters.begin(); it != clusters.end(); ++it, ++clusterIndex)
		{
#ifdef DEBUG_MODE
			DEBUG("Analyzing cluster: [" << it->second.x << "," << it->second.y << "]" << endl);
#endif
			
			const vector<int>::const_iterator& pendingCluster = find(pendingClusters.begin(),pendingClusters.end(),clusterIndex);
			
			/// This cluster has no observation associated, so we are keeping it to figure out if it is actually a real object.
			if (pendingCluster != pendingClusters.end())
			{
#ifdef DEBUG_MODE
				DEBUG("Pending cluster: [" << it->second.x << "," << it->second.y << "]" << endl);
#endif
				
				continue;
			}
			
			const vector<int>::const_iterator& deletingCluster = find(deletingClusters.begin(),deletingClusters.end(),clusterIndex);
			
			/// This cluster has to be deleted (it will happen later).
			if (deletingCluster != deletingClusters.end())
			{
#ifdef DEBUG_MODE
				DEBUG("Deleting cluster: [" << it->second.x << "," << it->second.y << "]" << endl);
#endif
				
				continue;
			}
			
			minDistance = FLT_MAX;
			index = -1;
			
			for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it2 = estimatedTargetModelsWithIdentity.begin(); it2 != estimatedTargetModelsWithIdentity.end(); ++it2)
			{
				const vector<int>::const_iterator& e = find(estimationsValid.begin(),estimationsValid.end(),it2->first);
				
				/// The estimation has been already associated to a cluster.
				if (e != estimationsValid.end()) continue;
				
				const Point2f& estimation = it2->second.first.observation.getCartesian();
				
				distance = sqrt(((it->second.x - estimation.x) * (it->second.x - estimation.x)) + ((it->second.y - estimation.y) * (it->second.y - estimation.y)));
				
				if (distance < minDistance)
				{
					index = it2->first;
					minDistance = distance;
				}
			}
			
			bool multipleObjectsTooCloseEachOther;
			
			multipleObjectsTooCloseEachOther = false;
			
			/// Found a close estimation, so we update the target with identity.
			if (minDistance < closenessThreshold)
			{
				stringstream indexStream;
				bool found;
				
				const map<int,pair<ObjectSensorReading::Observation,Point2f> >::iterator& estimation = estimatedTargetModelsWithIdentity.find(index);
				
#ifdef DEBUG_MODE
				ERR("Updating estimation (" << index << ") close to cluster: [" << it->second.x << "," << it->second.y << "]" << endl);
#endif
				
				found = false;
				
				indexStream << index;
				
				for (vector<pair<string,pair<ObjectSensorReading::Observation,Point2f> > >::const_iterator it2 = groupEstimatedTargetModelsWithIdentity.begin(); it2 != groupEstimatedTargetModelsWithIdentity.end(); ++it2)
				{
					if ((it2->first.find(" ") != string::npos) && (it2->first.find(indexStream.str()) != string::npos))
					{
						found = true;
						
						break;
					}
				}
				
				if (!found)
				{
					map<int,pair<ObjectSensorReading::Observation,Point2of> >::const_iterator obsMapping = observationsMapping.find(clusterIndex);
					
					/// Updating model using the associated observation.
					if (obsMapping != observationsMapping.end())
					{
						ObjectSensorReading::Model model;
						Point2f newMovement(FLT_MIN,FLT_MIN);
						
						const map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator& previousEstimation = estimatedTargetModelsWithIdentityPreviousIteration.find(index);
						
						if (previousEstimation != estimatedTargetModelsWithIdentityPreviousIteration.end())
						{
							Point2f displacement;
							
							displacement.x = ((obsMapping->second.first.head.x - previousEstimation->second.first.head.x) / velocityStabilizationFactor);
							displacement.y = ((obsMapping->second.first.head.y - previousEstimation->second.first.head.y) / velocityStabilizationFactor);
							
							if (sqrt((displacement.x * displacement.x) + (displacement.y * displacement.y)) < closenessThreshold)
							{
								Utils::EstimationDirection currentDirection, previousDirection;
								
								currentDirection = Utils::calculateEstimationDirection(displacement);
								previousDirection = Utils::calculateEstimationDirection(previousEstimation->second.first.model.velocity);
								
								/// Calculating the movement using the point on the floor (bottom-center of the bounding box).
								displacement.x = ((obsMapping->second.first.observation.getCartesian().x - previousEstimation->second.first.observation.getCartesian().x) / velocityStabilizationFactor);
								displacement.y = ((obsMapping->second.first.observation.getCartesian().y - previousEstimation->second.first.observation.getCartesian().y) / velocityStabilizationFactor);
								
								/// Giving more weight to the current estimated velocity.
								if (currentDirection == previousDirection)
								{
									newMovement.x = (previousEstimation->second.first.model.velocity.x * dt * 3 + displacement.x) / 4;
									newMovement.y = (previousEstimation->second.first.model.velocity.y * dt * 3 + displacement.y) / 4;
								}
								/// I am not sure what is going on. So, I choose a conservative strategy.
								else
								{
									newMovement.x = (previousEstimation->second.first.model.velocity.x * dt * 4 + displacement.x) / 5;
									newMovement.y = (previousEstimation->second.first.model.velocity.y * dt * 4 + displacement.y) / 5;
								}
								
								const map<int,Point2f>::iterator& movements = estimatedTargetMovements.find(index);
								
								if (movements != estimatedTargetMovements.end()) movements->second = newMovement;
								else estimatedTargetMovements.insert(make_pair(index,newMovement));
							}
							else
							{
								Point2f displacementMismatch;
								int mismatchedClusterIndex;
								
								minDistance = FLT_MAX;
								mismatchedClusterIndex = -1;
								
								for (unsigned int w = 0/*(clusterIndex + 1)*/; w < observationsMapping.size(); ++w)
								{
									const map<int,pair<ObjectSensorReading::Observation,Point2of> >::const_iterator& mismatchedObsMapping = observationsMapping.find(w);
									
									if (obsMapping != observationsMapping.end())
									{
										displacementMismatch.x = (mismatchedObsMapping->second.first.head.x - previousEstimation->second.first.head.x);
										displacementMismatch.y = (mismatchedObsMapping->second.first.head.y - previousEstimation->second.first.head.y);
										
										distance = sqrt((displacementMismatch.x * displacementMismatch.x) + (displacementMismatch.y * displacementMismatch.y));
										
										if (distance < minDistance)
										{
											minDistance = distance;
											mismatchedClusterIndex = w;
										}
									}
								}
								
								if (minDistance <= closenessThreshold)
								{
									const map<int,pair<ObjectSensorReading::Observation,Point2of> >::const_iterator& mismatchedObsMapping = observationsMapping.find(mismatchedClusterIndex);
									
									obsMapping = mismatchedObsMapping;
									
									displacementMismatch.x = ((mismatchedObsMapping->second.first.head.x - previousEstimation->second.first.head.x) / velocityStabilizationFactor);
									displacementMismatch.y = ((mismatchedObsMapping->second.first.head.y - previousEstimation->second.first.head.y) / velocityStabilizationFactor);
									
									Utils::EstimationDirection currentDirection, previousDirection;
									
									currentDirection = Utils::calculateEstimationDirection(displacementMismatch);
									previousDirection = Utils::calculateEstimationDirection(previousEstimation->second.first.model.velocity);
									
									/// Calculating the movement using the point on the floor (bottom-center of the bounding box).
									displacementMismatch.x = ((mismatchedObsMapping->second.first.observation.getCartesian().x - previousEstimation->second.first.observation.getCartesian().x) / velocityStabilizationFactor);
		                            displacementMismatch.y = ((mismatchedObsMapping->second.first.observation.getCartesian().y - previousEstimation->second.first.observation.getCartesian().y) / velocityStabilizationFactor);
									
									/// Giving more weight to the current estimated velocity.
									if (currentDirection == previousDirection)
									{
										newMovement.x = (previousEstimation->second.first.model.velocity.x * dt * 3 + displacementMismatch.x) / 4;
										newMovement.y = (previousEstimation->second.first.model.velocity.y * dt * 3 + displacementMismatch.y) / 4;
									}
									/// I am not sure what is going on. So, I choose a conservative strategy.
									else
									{
										newMovement.x = (previousEstimation->second.first.model.velocity.x * dt * 4 + displacementMismatch.x) / 5;
										newMovement.y = (previousEstimation->second.first.model.velocity.y * dt * 4 + displacementMismatch.y) / 5;
									}
									
									const map<int,Point2f>::iterator& movements = estimatedTargetMovements.find(index);
									
									if (movements != estimatedTargetMovements.end()) movements->second = newMovement;
									else estimatedTargetMovements.insert(make_pair(index,newMovement));
								}
								else multipleObjectsTooCloseEachOther = true;
							}
						}
						
						estimation->second.first.observation.rho = it->second.mod();
						estimation->second.first.observation.theta = atan2(it->second.y,it->second.x);
						estimation->second.first.head.x = obsMapping->second.first.head.x;
						estimation->second.first.head.y = obsMapping->second.first.head.y;
						estimation->second.first.sigma = Utils::calculateSigmaParticles(it->first,it->second);
						estimation->second.second = estimation->second.first.sigma;
						
						model.barycenter = obsMapping->second.first.model.barycenter;
						model.boundingBox.first.x = (estimation->second.first.model.boundingBox.first.x * 3 + obsMapping->second.first.model.boundingBox.first.x) / 4;
						model.boundingBox.first.y = (estimation->second.first.model.boundingBox.first.y * 3 + obsMapping->second.first.model.boundingBox.first.y) / 4;
						model.boundingBox.second.x = (estimation->second.first.model.boundingBox.second.x * 3 + obsMapping->second.first.model.boundingBox.second.x) / 4;
						model.boundingBox.second.y = (estimation->second.first.model.boundingBox.second.y * 3 + obsMapping->second.first.model.boundingBox.second.y) / 4;
						model.height = (estimation->second.first.model.height * 3 + obsMapping->second.first.model.height) / 4;
						model.width = (estimation->second.first.model.width * 3 + obsMapping->second.first.model.width) / 4;
						
						for (int i = 0; i < ObjectSensorReading::Model::HISTOGRAM_VECTOR_LENGTH; ++i)
						{
							model.histograms[0][i] = obsMapping->second.first.model.histograms[0][i];
							model.histograms[1][i] = obsMapping->second.first.model.histograms[1][i];
							model.histograms[2][i] = obsMapping->second.first.model.histograms[2][i];
						}
						
						/// Checking if we were able to calculate the displacement.
						if ((newMovement.x != FLT_MIN) || multipleObjectsTooCloseEachOther)
						{
							Point2f instantVelocity;
							float averagedVelocityX, averagedVelocityY;
							
							if (multipleObjectsTooCloseEachOther)
							{
								if (fabs(previousEstimation->second.first.model.velocity.x) < 0.2)
								{
									/// Around 1 m/s in the opposite direction.
									instantVelocity.x = (previousEstimation->second.first.model.velocity.x > 0) ? -1 : 1;
								}
								else instantVelocity.x = previousEstimation->second.first.model.velocity.x * -1;
								
								if (fabs(previousEstimation->second.first.model.velocity.y) < 0.2)
								{
									/// Around 1 m/s in the opposite direction.
									instantVelocity.y = (previousEstimation->second.first.model.velocity.y > 0) ? -1 : 1;
								}
								else instantVelocity.y = previousEstimation->second.first.model.velocity.y * -1;
								
								const PointWithVelocity& newPose = Utils::estimatedPosition(estimation->second.first.observation.getCartesian(),instantVelocity,dt);
								
								estimation->second.first.observation.rho = sqrt((newPose.pose.x * newPose.pose.x) + (newPose.pose.y * newPose.pose.y));
								estimation->second.first.observation.theta = atan2(newPose.pose.y,newPose.pose.x);
							}
							else
							{
								instantVelocity.x = ((obsMapping->second.first.head.x - previousEstimation->second.first.head.x) / velocityStabilizationFactor);
								instantVelocity.y = ((obsMapping->second.first.head.y - previousEstimation->second.first.head.y) / velocityStabilizationFactor);
							}
							
							model.velocity.x = instantVelocity.x / dt;
							model.velocity.y = instantVelocity.y / dt;
							
							if (model.velocity.x > modelLinearVelocity) model.velocity.x = previousEstimation->second.first.model.averagedVelocity.x;
							else if (model.velocity.x < -modelLinearVelocity) model.velocity.x = -previousEstimation->second.first.model.averagedVelocity.x;
							
							if (model.velocity.y > modelLinearVelocity) model.velocity.y = previousEstimation->second.first.model.averagedVelocity.y;
							else if (model.velocity.y < -modelLinearVelocity) model.velocity.y = -previousEstimation->second.first.model.averagedVelocity.y;
							
							const map<int,list<pair<float,float> > >::iterator& averagedVelocity = averagedVelocities.find(estimation->first);
							
							if (averagedVelocity == averagedVelocities.end())
							{
								averagedVelocityX = model.velocity.x;
								averagedVelocityY = model.velocity.y;
								
								list<pair<float,float> > averagedVelocitiesVector;
								
								averagedVelocitiesVector.push_back(make_pair(averagedVelocityX,averagedVelocityY));
								
								averagedVelocities.insert(make_pair(estimation->first,averagedVelocitiesVector));
							}
							else
							{
								averagedVelocity->second.push_back(make_pair(model.velocity.x,model.velocity.y));
								
								averagedVelocityX = 0;
								averagedVelocityY = 0;
								
								for (list<pair<float,float> >::const_iterator it2 = averagedVelocity->second.begin(); it2 != averagedVelocity->second.end(); ++it2)
								{
									averagedVelocityX += it2->first;
									averagedVelocityY += it2->second;
								}
								
								averagedVelocityX /= averagedVelocity->second.size();
								averagedVelocityY /= averagedVelocity->second.size();
							}
							
							model.averagedVelocity.x = averagedVelocityX;
							model.averagedVelocity.y = averagedVelocityY;
						}
						
						estimation->second.first.model = model;
						
						for (vector<pair<ObjectSensorReading::Model,int> >::iterator it2 = targetModels.begin(); it2 != targetModels.end(); ++it2)
						{
							if (it2->second == estimation->first)
							{
								it2->first = model;
								
								break;
							}
						}
					}
					else
					{
						if (fabs(estimation->second.first.model.velocity.x) < 0.2)
						{
							/// Around 1 m/s in the opposite direction.
							estimation->second.first.model.velocity.x = (estimation->second.first.model.velocity.x > 0) ? -1 : 1;
						}
						else estimation->second.first.model.velocity.x = estimation->second.first.model.velocity.x * -1;
						
						if (fabs(estimation->second.first.model.velocity.y) < 0.2)
						{
							/// Around 1 m/s in the opposite direction.
							estimation->second.first.model.velocity.y = (estimation->second.first.model.velocity.y > 0) ? -1 : 1;
						}
						else estimation->second.first.model.velocity.y = estimation->second.first.model.velocity.y * -1;
						
						const PointWithVelocity& newPose = Utils::estimatedPosition(estimation->second.first.observation.getCartesian(),estimation->second.first.model.velocity,dt);
						
						estimation->second.first.observation.rho = sqrt((newPose.pose.x * newPose.pose.x) + (newPose.pose.y * newPose.pose.y));
						estimation->second.first.observation.theta = atan2(newPose.pose.y,newPose.pose.x);
					}
					
					const map<int,Timestamp>::iterator& estimationTime = estimationsUpdateTime.find(index);
					
					/// Updating the timestamp of the estimation.
					estimationTime->second.setToNow();
				}
				/// The estimation is part of a group so the position is updated using its estimated velocity.
				else
				{
					const PointWithVelocity& newPose = Utils::estimatedPosition(estimation->second.first.observation.getCartesian(),estimation->second.first.model.velocity,dt);
					
					estimation->second.first.observation.rho = sqrt((newPose.pose.x * newPose.pose.x) + (newPose.pose.y * newPose.pose.y));
					estimation->second.first.observation.theta = atan2(newPose.pose.y,newPose.pose.x);
				}
				
				estimationsValid.push_back(index);
			}
			/// No closes estimations have been found, so we are searching for an existing model of the cluster.
			else
			{
				minDistance = FLT_MAX;
				index = -1;
				i = 0;
				
				for (vector<ObjectSensorReading::Observation>::iterator it2 = obs.begin(); it2 != obs.end(); ++it2, ++i)
				{
					const Point2f& o = it2->observation.getCartesian();
					
					distance = sqrt(((it->second.x - o.x) * (it->second.x - o.x)) + ((it->second.y - o.y) * (it->second.y - o.y)));
					
					if (distance < minDistance)
					{
						minDistance = distance;
						index = i;
					}
				}
				
				/// An observation close to the cluster has been found. Basically this branch is executed when a new cluster become an estimation.
				if (minDistance < closenessThreshold)
				{
#ifdef DEBUG_MODE
					ERR("New estimation [" << it->second.x << "," << it->second.y << "] close to cluster: [" << it->second.x << "," << it->second.y << "]" << endl);
#endif
					
					ObjectSensorReading::Observation target;
					ObjectSensorReading::Model model;
					float minBhattacharyyaDistance;
					int targetIdentity;
					
					/// A new observation has been promoted and it is not contained into the observation mapping.
					--clusterIndex;
					
					model = obs.at(index).model;
					
					target.observation = PolarPoint(atan2(it->second.y,it->second.x),it->second.mod());
					target.head = obs.at(index).head;
					target.sigma = Utils::calculateSigmaParticles(it->first,it->second);
					
					/// Observation used and no longer needed.
					obs.erase(obs.begin() + index);
					
					minBhattacharyyaDistance = FLT_MAX;
					index = -1;
					i = 0;
					
					for (vector<pair<ObjectSensorReading::Model,int> >::const_iterator it2 = targetModels.begin(); it2 != targetModels.end(); ++it2, ++i)
					{
						pair<bool,float> bhattacharyyaComparison = areModelsSimilar(model,it2->first);
						
						if (bhattacharyyaComparison.first)
						{
							if (bhattacharyyaComparison.second < minBhattacharyyaDistance)
							{
								/// Checking if the re-identification was unique. It could happen that two different objects are re-identified with the same model.
								if (estimatedTargetModelsWithIdentity.find(it2->second) == estimatedTargetModelsWithIdentity.end())
								{
									minBhattacharyyaDistance = bhattacharyyaComparison.second;
									index = i;
								}
							}
						}
					}
					
					/// An existing model has been found.
					if (index != -1)
					{
						/// Since the re-identification has been successfully, we can just get the identity and the color histogram since all the other information are no longer valid.
						target.model = model;
						
						for (int i = 0; i < ObjectSensorReading::Model::HISTOGRAM_VECTOR_LENGTH; ++i)
						{
							target.model.histograms[0][i] = targetModels.at(index).first.histograms[0][i];
							target.model.histograms[1][i] = targetModels.at(index).first.histograms[1][i];
							target.model.histograms[2][i] = targetModels.at(index).first.histograms[2][i];
						}
						
						targetIdentity = targetModels.at(index).second;
						
						/// Checking if the re-identification was unique. It could happen that two different objects are re-identified with the same model.
						if (estimatedTargetModelsWithIdentity.find(targetIdentity) != estimatedTargetModelsWithIdentity.end())
						{
							targetIdentity = ++maxIdentityNumber;
							
							targetModels.push_back(make_pair(target.model,targetIdentity));
						}
					}
					else
					{
						target.model = model;
						targetIdentity = ++maxIdentityNumber;
						
						targetModels.push_back(make_pair(target.model,targetIdentity));
					}
					
					estimatedTargetModelsWithIdentity.insert(make_pair(targetIdentity,make_pair(target,target.sigma)));
					estimationsUpdateTime.insert(make_pair(targetIdentity,Timestamp()));
					
					estimationsValid.push_back(targetIdentity);
				}
			}
		}
		
		i = 0;
		
		/// Deleting clusters.
		for (vector<pair<PoseParticleVector,Point2of> >::iterator it = clusters.begin(); it != clusters.end(); ++i)
		{
			const vector<int>::const_iterator& deletingCluster = find(deletingClusters.begin(),deletingClusters.end(),i);
			
			/// This cluster has to be deleted.
			if (deletingCluster != deletingClusters.end()) it = clusters.erase(it);
			else ++it;
		}
		
		/// Updating timestamp of the estimations that are forming a group.
		for (vector<pair<string,pair<ObjectSensorReading::Observation,Point2f> > >::const_iterator it = groupEstimatedTargetModelsWithIdentity.begin(); it != groupEstimatedTargetModelsWithIdentity.end(); ++it)
		{
			vector<int> groupIdentities;
			int identity;
			bool updateEstimationTimestamp;
			
			s.clear();
			s.str("");
			
			s << it->first;
			
			while (s >> identity)
			{
				groupIdentities.push_back(identity);
			}
			
			if (groupIdentities.size() > 1)
			{
				updateEstimationTimestamp = false;
				
				for (vector<int>::const_iterator it2 = groupIdentities.begin(); !updateEstimationTimestamp && (it2 != groupIdentities.end()); ++it2)
				{
					const map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator est = estimatedTargetModelsWithIdentity.find(*it2);
					
					for (vector<ObjectSensorReading::Observation>::iterator it3 = obs.begin(); it3 != obs.end(); ++it3)
					{
						if (Utils::isTargetNear(est->second.first.observation.getCartesian(),it3->observation.getCartesian(),closenessThreshold))
						{
							updateEstimationTimestamp = true;
							
							break;
						}
					}
				}
				
				if (updateEstimationTimestamp)
				{
					for (vector<int>::const_iterator it2 = groupIdentities.begin(); it2 != groupIdentities.end(); ++it2)
					{
						const map<int,Timestamp>::iterator& estimationTime = estimationsUpdateTime.find(*it2);
						
						/// Updating the timestamp of the estimation.
						estimationTime->second.setToNow();
					}
				}
			}
		}
		
		/// Removing all the estimations that have not been associated for a period equals to timeToWaitBeforeDeleting.
		for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::iterator it = estimatedTargetModelsWithIdentity.begin(); it != estimatedTargetModelsWithIdentity.end(); )
		{
			const vector<int>::const_iterator& e = find(estimationsValid.begin(),estimationsValid.end(),it->first);
			
			if (e != estimationsValid.end()) ++it;
			else
			{
				const map<int,Timestamp>::iterator& estimationTime = estimationsUpdateTime.find(it->first);
				
				if (((long) currentTimestamp - (long) estimationTime->second.getMsFromMidnight()) > (long) timeToWaitBeforeDeleting)
				{
#ifdef DEBUG_MODE
					INFO("Deleting old estimation (" << it->first << ") -> [" << it->second.first.observation.getCartesian().x << "," << it->second.first.observation.getCartesian().y << "], " <<
						 "time = " << ((long) currentTimestamp - (long) estimationTime->second.getMsFromMidnight()) << " ms" << endl);
#endif
					
					estimatedTargetModelsWithIdentity.erase(it++);
					estimationsUpdateTime.erase(estimationTime);
				}
				else
				{
#ifdef DEBUG_MODE
					WARN("Keeping old estimation (" << it->first << ") -> [" << it->second.first.observation.getCartesian().x << "," << it->second.first.observation.getCartesian().y << "]" << endl);
#endif
					
					if ((fabs(it->second.first.model.velocity.x) <= modelLinearVelocity) && (fabs(it->second.first.model.velocity.y) <= modelLinearVelocity))
					{
						const PointWithVelocity& newPose = Utils::estimatedPosition(it->second.first.observation.getCartesian(),it->second.first.model.velocity,dt);
						
						it->second.first.observation.rho = sqrt((newPose.pose.x * newPose.pose.x) + (newPose.pose.y * newPose.pose.y));
						it->second.first.observation.theta = atan2(newPose.pose.y,newPose.pose.x);
					}
					
					++it;
				}
			}
		}
		
		/// Either removing the averaged velocity of an old estimation or removing farest averaged velocity of the estimation.
		for (map<int,list<pair<float,float> > >::iterator it = averagedVelocities.begin(); it != averagedVelocities.end(); )
		{
			const map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator& estimation = estimatedTargetModelsWithIdentity.find(it->first);
			
			if (estimation != estimatedTargetModelsWithIdentity.end())
			{
				if (it->second.size() == averagedVelocityWindow) it->second.erase(it->second.begin());
				
				++it;
			}
			else averagedVelocities.erase(it++);
		}
	}
	
	SENSORFILTER_FACTORY(ObjectParticleFilter)
}
