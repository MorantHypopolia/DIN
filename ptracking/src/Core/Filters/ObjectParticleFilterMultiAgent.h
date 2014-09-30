#pragma once

#include "../Filters/ObjectSensorReadingMultiAgent.h"
#include <Utils/Timestamp.h>
#include <Utils/Utils.h>
#include <Manfield/filters/particlefilter.h>

namespace PTracking
{
	/**
	 * @brief Forward declaration.
	 */
	class Clusterizer;
}

namespace PTracking
{
	/**
	 * @class ObjectParticleFilterMultiAgent
	 * 
	 * @brief Class that implements the particle filter used in the global estimation layer.
	 */
	class ObjectParticleFilterMultiAgent : public manfield::ParticleFilter
	{
		private:
			/**
			 * @brief map representing the estimations having both an identity and a model of the observation performed by the team of agents.
			 */
			std::map<int,std::pair<std::pair<ObjectSensorReading::Observation,Point2f>,std::pair<std::string,int> > > estimatedTargetModelsWithIdentityMultiAgent;
			
			/**
			 * @brief map representing the timestamp on which an estimation has been updated.
			 */
			std::map<int,Timestamp> estimationsMultiAgentUpdateTime;
			
			/**
			 * @brief pointer to the clustering algorithm.
			 */
			Clusterizer* clusterizer;
			
			/**
			 * @brief type of the clustering algortithm.
			 */
			std::string clusteringAlgorithm;
			
			/**
			 * @brief threshold used for the data association phase.
			 */
			float closenessThreshold;
			
			/**
			 * @brief timestamp of the current iteration.
			 */
			unsigned long currentTimestamp;
			
			/**
			 * @brief time to wait before deleting an estimation no longer associated to an estimation performed by the team of agents.
			 */
			unsigned int timeToWaitBeforeDeleting;
			
			/**
			 * @brief maximum identity value assigned to an estimation.
			 */
			int maxIdentityNumber;
			
			/**
			 * @brief number of the best particles exchanged by the agents.
			 */
			int bestParticlesNumber;
			
			/**
			 * @brief Function that adjusts the particle weight using the model given in input.
			 * 
			 * @param p reference to the particle to be updated.
			 * @param particlesTimestamp timestamp of the particles.
			 * @param currentTimestamp timestamp of the current iteration.
			 * @param model model used to update the particle weight.
			 * @param factor factor used to update the particle weight.
			 * 
			 * @return the updated particle.
			 */
			PoseParticle adjustWeight(const PoseParticle& p, unsigned long particlesTimestamp, unsigned long currentTimestamp, Utils::DecreaseModelFactor model, float factor) const;
			
			/**
			 * @brief Function that checks if two estimations, performed by two different agents, have the same direction.
			 * 
			 * @param e1 reference to the estimation performed by one agent.
			 * @param e2 reference to the estimation performed by the other agent.
			 * 
			 * @return \b true if the estimations have the same direction, \b false otherwise.
			 */
			bool isSameDirection(const pair<int,pair<ObjectSensorReading::Observation,Point2f> >& e1, const pair<int,pair<ObjectSensorReading::Observation,Point2f> >& e2) const;
			
			/**
			 * @brief Function that resets the weight of the particles by setting their value to 1.
			 */
			void resetWeight();
			
			/**
			 * @brief Function that updates the estimations having both an identity and a model performed by the agent.
			 * 
			 * @param readings reference to the observations gathered from the sensors between the previous and current iteration.
			 */
			void updateTargetIdentity(const std::vector<ObjectSensorReadingMultiAgent>& readings);
			
		public:
			/**
			 * @brief Constructor that takes the type of the particle filter as initialization value.
			 * 
			 * It initializes the type of the particle filter with the one given in input.
			 * 
			 * @param type particle filter type.
			 */
			ObjectParticleFilterMultiAgent(const std::string& type = "ObjectParticleFilterMultiAgent");
			
			/**
			 * @brief Destructor.
			 */
			~ObjectParticleFilterMultiAgent();
			
			/**
			 * @brief Function that reads a config file in order to initialize several configuration parameters.
			 * 
			 * @param filename file to be read.
			 */
			void configure(const std::string& filename);
			
			/**
			 * @brief Function that returns the pointer to the clustering algorithm.
			 * 
			 * @return the pointer to the clustering algorithm.
			 */
			inline Clusterizer* getClusterizer() { return clusterizer; }
			
			/**
			 * @brief Function that returns the estimations having both an identity and a model of the observation performed by the team of agents.
			 * 
			 * @return a reference to the estimations having both an identity and a model of the observation performed by the team of agents.
			 */
			inline const std::map<int,std::pair<std::pair<ObjectSensorReading::Observation,Point2f>,std::pair<std::string,int> > >& getEstimationsWithModel() const { return estimatedTargetModelsWithIdentityMultiAgent; }
			
			/**
			 * @brief Function that returns the sensor used by the agent.
			 * 
			 * @return a reference to the sensor used by the agent.
			 */
			inline const BasicSensor& getSensor() const { return *static_cast<const BasicSensor*>(ParticleFilter::getSensor()); }
			
			/**
			 * @brief Function that creates the new likelihood distribution using the estimations received by the team of agents.
			 * 
			 * @param readings reference to the estimations received by the other agents between the previous and current iteration.
			 */
			void observe(const std::vector<ObjectSensorReadingMultiAgent>& readings);
			
			/**
			 * @brief Macro that defines the default clone function.
			 */
			FILTER_DEFAULT_CLONE(ObjectParticleFilterMultiAgent)
	};
}
