#pragma once

#include <CGAL/QP_models.h>
#include <CGAL/MP_Float.h>

/**
 * @brief Forward declaration.
 */
class Symbolic;

namespace PTracking
{
	/**
	 * @class IRLearner
	 * 
	 * @brief Class that implements an inverse reinforcement learning algorithm.
	 */
	class IRLearner
	{
		private:
			typedef CGAL::Quadratic_program_from_mps<float> Program;
			
			/**
			 * @brief pointer to the problem to be solved.
			 */
			Program* problem;
			
			/**
			 * @brief the solution of the problem, if any.
			 */
			std::pair<float,std::vector<std::pair<std::string,float> > > solution;
			
		public:
			/**
			 * @brief Empty constructor.
			 */
			IRLearner();
			
			/**
			 * @brief Destructor.
			 */
			virtual ~IRLearner();
			
			/**
			 * @brief Function that generates the LP problem given the information of the current state.
			 * 
			 * @param filename file to be written.
			 * @param numberOfStates states of the problem.
			 * @param numberOfActions possible actions for a given state.
			 * @param probabilityMatrices reference to the transition probability matrix of each action.
			 * @param policy reference to the current optimal policy.
			 * @param gamma discount factor.
			 * @param lambda penalty factor.
			 */
			void generate(const std::string& filename, int numberOfStates, int numberOfActions, const std::vector<Symbolic>& probabilityMatrices, const Symbolic& policy, float gamma, float lambda) const;
			
			/**
			 * @brief Function that returns a reference to the solution of the problem.
			 * 
			 * @return reference to the solution of the problem.
			 */
			inline const std::pair<float,std::vector<std::pair<std::string,float> > >& getSolution() const { return solution; }
			
			/**
			 * @brief Function that loads a problem to be solved using the IRL algorithm.
			 * 
			 * @param filename file to be read.
			 */
			void load(const std::string& filename);
			
			/**
			 * @brief Function that solves the problem currently loaded.
			 */
			void solve();
			
			/**
			 * @brief Function that writes a LP problem dynamically generated.
			 * 
			 * @param filename file to be written.
			 * @param rows reference to the vector containing the rows of the problem.
			 * @param columns reference to the vector containing the columns of the problem.
			 * @param rhs reference to the vector containing the right-hand side vector of the problem.
			 * @param bounds reference to the vector containing the bounds of the variables.
			 */
			void write(const std::string& filename, const std::vector<std::pair<char,std::string> >& rows, const std::vector<std::pair<std::string,std::pair<std::string,float> > >& columns,
					   const std::vector<std::pair<std::string,float> >& rhs, const std::vector<std::pair<std::string,std::pair<std::string,float> > >& bounds) const;
	};
}
