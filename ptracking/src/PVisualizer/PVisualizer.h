#pragma once

#include <Core/Filters/ObjectSensorReading.h>

/**
 * @class PVisualizer
 * 
 * @brief @FIXME @FIXME @FIXME @FIXME @FIXME
 */
class PVisualizer
{
	private:
		/**
		 * @brief map containing the mapping between identity and color.
		 */
		std::map<int,std::pair<int,std::pair<int,int> > > colorMap;
		
		/**
		 * @brief Function that allows a clean exit intercepting the SIGINT signal.
		 */
		static void interruptCallback(int);
		
		/**
		 * @brief Function that reads the xml file containing all the estimations performed by PTracker.
		 * 
		 * @param estimationFile reference to the file containing all the estimations.
		 * 
		 * @return the data structure containing all the estimations.
		 */
		std::vector<std::pair<std::vector<int>,PTracking::ObjectSensorReading> > readEstimationFile(const std::string& estimationFile);
		
	public:
		/**
		 * @brief Empty constructor.
		 * 
		 * It registrers the SIGINT callback.
		 */
		PVisualizer();
		
		/**
		 * @brief Destructor.
		 */
		~PVisualizer();
		
		/**
		 * @brief Function that visualizes the PTracker results in the images using OpenCV. It stops when either has finished or Ctrl+C is pressed.
		 * 
		 * @param imagePath reference to the dataset image path.
		 * @param resultsXmlFile reference to the results xml file.
		 */
		void exec(const std::string& imagePath, const std::string& resultsXmlFile);
};
