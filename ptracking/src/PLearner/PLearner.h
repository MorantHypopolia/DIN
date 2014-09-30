#pragma once

#include <Core/IRLearner/IRLearner.h>
#include <Utils/Utils.h>

/**
 * @brief Forward declaration.
 */
class Symbolic;

/**
 * @class PLearner
 * 
 * @brief Class that uses an IRL method to solve a floating linear programming problem.
 */
class PLearner
{
	private:
		/**
		 * @brief vector containing the transition probability matrix of each possible action.
		 */
		std::vector<Symbolic> probabilityMatrices;
		
		/**
		 * @brief current optimal policy for the problem.
		 */
		Symbolic* policy;
		
		/**
		 * @brief IRL algorithm.
		 */
		PTracking::IRLearner learner;
		
		/**
		 * @brief discount factor.
		 */
		float gamma;
		
		/**
		 * @brief penalty factor.
		 */
		float lambda;
		
		/**
		 * @brief probability to move in a random direction instead of in the desired one.
		 */
		float probabilityRandomAction;
		
		/**
		 * @brief number of columns of the grid used for the IRL problem.
		 */
		int gridColumns;
		
		/**
		 * @brief number of rows of the grid used for the IRL problem.
		 */
		int gridRows;
		
		/**
		 * @brief number of possible actions in a given state of the grid.
		 */
		int numberOfActions;
		
		/**
		 * @brief Function that generates the probability matrix for the action given in input.
		 * 
		 * @param action for which the probability matrix have to be calculated.
		 * 
		 * @return the probability matrix for the action given in input.
		 */
		Symbolic generateProbabilityMatrix(PTracking::Utils::GridAction action) const;
		
		/**
		 * @brief Function that checks whether the agent can move in the desired state using the desired direction.
		 * 
		 * @param rowState row of the current state of the agent.
		 * @param columnState column of the current state of the agent.
		 * @param rowDesiredState row of the desired state of the agent.
		 * @param columnDesiredState column of the desired state of the agent.
		 * @param action desired direction of the agent.
		 * 
		 * @return a value representing the results of the movement.
		 */
		PTracking::Utils::GridActionResult tryMove(int rowState, int columnState, int rowDesiredState, int columnDesiredState, PTracking::Utils::GridAction action) const;
		
	public:
		/**
		 * @brief Empty constructor.
		 */
		PLearner();
		
		/**
		 * @brief Destructor.
		 */
		~PLearner();
		
		/**
		 * @brief Function that solves the problem currently loaded.
		 */
		void exec();
		
		/**
		 * @brief Function that generates the IRL problem given the current state.
		 * 
		 * @param filename file to be written.
		 */
		void generate(const std::string& filename) const;
		
		/**
		 * @brief Function that loads a problem to be solved using the IRL algorithm.
		 * 
		 * @param filename file to be read.
		 */
		void load(const std::string& filename);
};
