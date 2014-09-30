#pragma once

#include "../Sensors/BasicSensor.h"
#include <Utils/Point2of.h>
#include <Utils/PolarPoint.h>
#include <ThirdParty/GMapping/sensor/sensor_base/sensorreading.h>
#include <vector>

namespace PTracking
{
	/**
	 * @class ObjectSensorReading
	 * 
	 * @brief Class that defines the observations coming from the agent's sensors.
	 */
	class ObjectSensorReading : public GMapping::SensorReading
	{
		public:
			/**
			 * @struct Model
			 * 
			 * @brief Struct representing either the model of the observation or the model of the estimation.
			 */
			struct Model
			{
				/**
				 * @brief length of the histogram vector.
				 */
				static const int HISTOGRAM_VECTOR_LENGTH = 256;
				
				/**
				 * @brief boundingBox of the observation/estimation.
				 */
				std::pair<Point2f,Point2f> boundingBox;
				
				/**
				 * @brief width of the observation/estimation.
				 */
				int width;
				
				/**
				 * @brief height of the observation/estimation.
				 */
				int height;
				
				/**
				 * @brief barycenter of the observation/estimation.
				 */
				int barycenter;
				
				/**
				 * @brief histograms of the observation/estimation.
				 */
				float histograms[3][HISTOGRAM_VECTOR_LENGTH];
				
				/**
				 * @brief averaged velocity of the estimation.
				 */
				Point2f averagedVelocity;
				
				/**
				 * @brief velocity of the estimation.
				 */
				Point2f velocity;
				
				Model() : barycenter(-1)
				{
					for (int i = 0; i < ObjectSensorReading::Model::HISTOGRAM_VECTOR_LENGTH; ++i)
					{
						histograms[0][i] = 0;
						histograms[1][i] = 0;
						histograms[2][i] = 0;
					}
				}
				
				Model& operator= (const Model& other)
				{
					boundingBox = other.boundingBox;
					width = other.width;
					height = other.height;
					barycenter = other.barycenter;
					
					for (int i = 0; i < HISTOGRAM_VECTOR_LENGTH; ++i)
					{
						histograms[0][i] = other.histograms[0][i];
						histograms[1][i] = other.histograms[1][i];
						histograms[2][i] = other.histograms[2][i];
					}
					
					averagedVelocity.x = other.averagedVelocity.x;
					averagedVelocity.y = other.averagedVelocity.y;
					
					velocity.x = other.velocity.x;
					velocity.y = other.velocity.y;
					
					return *this;
				}
			};
			
			/**
			 * @struct Observation
			 * 
			 * @brief Struct representing either an observation provided by the agent's sensors or an estimation performed by the agent.
			 */
			struct Observation
			{
				/**
				 * @brief polar point representing an observation/estimation.
				 */
				PolarPoint observation;
				
				/**
				 * @brief point representing the head of the observation/estimation.
				 */
				Point2f head;
				
				/**
				 * @brief standard deviation of the observation/estimation.
				 */
				Point2f sigma;
				
				/**
				 * @brief model of the estimation.
				 */
				Model model;
			};
			
			/**
			 * @brief Empty constructor.
			 */
			ObjectSensorReading();
			
			/**
			 * @brief Destructor.
			 */
			~ObjectSensorReading();
			
			/**
			 * @brief Function that returns either the const vector representing the observation coming from the agent's sensors or the const estimations performed by the agent.
			 * 
			 * @return a reference to either the const vector of the observation coming from the agent's sensors or the const estimations performed by the agent.
			 */
			std::vector<Observation>& getObservations() { return observations; }
			
			/**
			 * @brief Function that returns either the vector representing the observation coming from the agent's sensors or the estimations performed by the agent.
			 * 
			 * @return a reference to either the vector of the observation coming from the agent's sensors or the estimations performed by the agent.
			 */
			const std::vector<Observation>& getObservations() const { return observations; }
			
			/**
			 * @brief Function that returns the position of the agent when the observations have been acquired.
			 * 
			 * @return a reference to the position of the agent when the observations have been acquired.
			 */
			const Point2of& getObservationsAgentPose() const { return observationsAgentPose; }
			
			/**
			 * @brief Function that returns the sensor used by the agent.
			 * 
			 * @return a reference to the sensor used by the agent.
			 */
			const BasicSensor& getSensor() const { return sensor; }
			
			/**
			 * @brief Function that updates the vector of observations coming from the agent's sensors.
			 * 
			 * @param obs reference to the vector of observations coming from the agent's sensors.
			 */
			void setObservations(const std::vector<Observation>& obs);
			
			/**
			 * @brief Function that updates the circular buffer of the observations coming from the agent's sensors by checking a maximum threshold distance.
			 * 
			 * @param targetPoints pointer to the observations provided by the agent's sensors.
			 * @param currentTargetIndex current index of the circular buffer.
			 * @param lastCurrentTargetIndex last current index of the circular buffer.
			 * @param lastNTargetPerception size of the circular buffer.
			 * @param worldX reference to the world x admissible range.
			 * @param worldY reference to the world y admissible range.
			 */
			void setObservations(Observation targetPoints[], int currentTargetIndex, int lastCurrentTargetIndex, int lastNTargetPerception, const Point2f& worldX, const Point2f& worldY);
			
			/**
			 * @brief Function that updates the position of the agent when the observations have been acquired.
			 * 
			 * @param pose reference to the new position of the agent.
			 */
			void setObservationsAgentPose(const Point2of& pose);
			
			/**
			 * @brief Function that sets a new sensor.
			 * 
			 * @param s reference to a new sensor.
			 */
			void setSensor(const BasicSensor& s);
			
		private:
			/**
			 * @brief vector representing either the observation coming from the agent's sensors or the estimations performed by the agent.
			 */
			std::vector<Observation> observations;
			
			/**
			 * @brief the last position of the agent when the observations have been acquired.
			 */
			Point2of observationsAgentPose;
			
			/**
			 * @brief sensor used by the agent.
			 */
			BasicSensor sensor;
	};
}
