#include "PLearner.h"
#include <Utils/Timestamp.h>
#include <Utils/Utils.h>
#include <Manfield/configfile/configfile.h>
#include <Manfield/utils/debugutils.h>
#include <ThirdParty/SymbolicC++/symbolicc++.h>
#include <cfloat>

/// Uncomment if you want to analyze the time needed to solve the IRL problem.
#define ANALYZE_COMPUTATION_TIME

/// Uncomment if you want to print the matrices of the IRL problem.
//#define PRINT_MATRICES

using namespace std;
using GMapping::ConfigFile;
using PTracking::Timestamp;
using PTracking::Utils;

PLearner::PLearner() : policy(0)
{
	ConfigFile fCfg;
	string key, section;
	int numberOfStates;
	
	if (!fCfg.read(string(getenv("PTracking_ROOT")) + string("/../config/learner.cfg")))
	{
		ERR("Error reading file '" << string(getenv("PTracking_ROOT")) << string("/../config/learner.cfg") << "' for PLearner configuration. Exiting..." << endl);
		
		exit(-1);
	}
	
	try
	{
		section = "PLearner";
		
		key = "gridRows";
		gridRows = fCfg.value(section,key);
		
		key = "gridColumns";
		gridColumns = fCfg.value(section,key);
		
		key = "gamma";
		gamma = fCfg.value(section,key);
		
		key = "lambda";
		lambda = fCfg.value(section,key);
		
		key = "numberOfActions";
		numberOfActions = fCfg.value(section,key);
		
		key = "probabilityRandomAction";
		probabilityRandomAction = fCfg.value(section,key);
	}
	catch (...)
	{
		ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
		
		exit(-1);
	}
	
	ERR(endl << "******************************************************" << endl);
	DEBUG("PLearner parameters:" << endl);
	
	INFO("\tGrid size: ");
	WARN(gridRows << " x " << gridColumns << endl);
	
	INFO("\tNumber of possible actions: ");
	WARN(numberOfActions << endl);
	
	INFO("\tLambda: ");
	WARN(lambda << endl);
	
	INFO("\tGamma: ");
	WARN(gamma << endl);
	
	INFO("\tProbability to perform a random action: ");
	WARN(probabilityRandomAction << endl);
	
	ERR("******************************************************" << endl << endl);
	
	if (numberOfActions == 4)
	{
#ifdef ANALYZE_COMPUTATION_TIME
		Timestamp initial;
#endif
		
		INFO("Generating transition probability matrices for all possible actions..." << flush);
		
		probabilityMatrices.push_back(generateProbabilityMatrix(Utils::North));
		probabilityMatrices.push_back(generateProbabilityMatrix(Utils::East));
		probabilityMatrices.push_back(generateProbabilityMatrix(Utils::South));
		probabilityMatrices.push_back(generateProbabilityMatrix(Utils::West));
		
#ifdef ANALYZE_COMPUTATION_TIME
		INFO("done. (" << (Timestamp() - initial).getMs() << " ms)" << endl);
#else
		INFO("done." << endl);
#endif
	}
	else
	{
		ERR("Sorry, but the number of possible actions given in input ");
		INFO("(" << numberOfActions << ") ");
		ERR("is not supported yet. Please set this parameter to ");
		INFO("4");
		ERR(". Exiting..." << endl);
		
		exit(0);
	}
	
	numberOfStates = gridRows * gridColumns;
	
	policy = new Symbolic("Pa_*",numberOfStates,numberOfStates);
	
	for (int i = 0; i < numberOfStates; ++i)
	{
		for (int j = 0; j < numberOfStates; ++j)
		{
			(*policy)(i,j) = 0;
		}
	}
	
#if 1
	(*policy)(24,1) = 0.5;
	(*policy)(24,5) = 0.5;
	(*policy)(23,2) = 0.5;
	(*policy)(23,6) = 0.5;
	(*policy)(22,3) = 1;
	(*policy)(21,4) = 0.5;
	(*policy)(21,8) = 0.5;
	(*policy)(20,9) = 1;
	(*policy)(19,10) = 1;
	(*policy)(18,7) = 0.5;
	(*policy)(18,11) = 0.5;
	(*policy)(17,8) = 0.5;
	(*policy)(17,12) = 0.5;
	(*policy)(16,13) = 1;
	(*policy)(15,14) = 1;
	(*policy)(14,15) = 1;
	(*policy)(13,16) = 1;
	(*policy)(12,17) = 1;
	(*policy)(11,18) = 1;
	(*policy)(10,19) = 1;
	(*policy)(9,16) = 0.5;
	(*policy)(9,20) = 0.5;
	(*policy)(8,17) = 1;
	(*policy)(7,18) = 1;
	(*policy)(6,23) = 1;
	(*policy)(5,24) = 1;
	(*policy)(4,21) = 1;
	(*policy)(3,22) = 1;
	(*policy)(2,23) = 1;
	(*policy)(1,24) = 1;
	
	/// Deterministic policy.
	/* (*policy)(24,1) = 1;
	(*policy)(23,2) = 1;
	(*policy)(22,3) = 1;
	(*policy)(21,4) = 1;
	(*policy)(20,9) = 1;
	(*policy)(15,14) = 1;
	(*policy)(10,19) = 1;
	(*policy)(6,24) = 1;*/
	
#else
	(*policy)(0,0) = 0;
	(*policy)(0,1) = 0;
	(*policy)(0,2) = 0;
	(*policy)(0,3) = 0;
	
	(*policy)(1,0) = 0;
	(*policy)(1,1) = 0;
	(*policy)(1,2) = 0;
	(*policy)(1,3) = 1;
	
	(*policy)(2,0) = 0;
	(*policy)(2,1) = 0;
	(*policy)(2,2) = 0;
	(*policy)(2,3) = 1;
	
	(*policy)(3,0) = 0;
	(*policy)(3,1) = 0.5;
	(*policy)(3,2) = 0.5;
	(*policy)(3,3) = 0;
#endif
	
#ifdef PRINT_MATRICES
	INFO(endl << "Policy:" << endl);
	
	for (int i = 0; i < numberOfStates; ++i)
	{
		for (int j = 0; j < numberOfStates; ++j)
		{
			DEBUG((*policy)(i,j) << " ");
		}
		
		INFO(endl);
	}
	
	INFO(endl);
#endif
}

PLearner::~PLearner()
{
	if (policy != 0) delete policy;
}

void PLearner::exec()
{
#ifdef ANALYZE_COMPUTATION_TIME
	Timestamp initial;
#endif
	
	ERR("Solving IRL problem..." << flush);
	
	learner.solve();
	
#ifdef ANALYZE_COMPUTATION_TIME
	ERR("done. (" << (Timestamp() - initial).getMs() << " ms)" << endl);
#else
	ERR("done." << endl);
#endif
	
	const pair<float,vector<pair<string,float> > >& solution = learner.getSolution();
	
	if (solution.first == -FLT_MAX)
	{
		INFO("The problem has no solution." << endl);
	}
	else
	{
		ofstream gnuplotFile("gnuplot.txt");
		int i, j;
		
		DEBUG(endl << "Variables:" << endl);
		
		i = 1;
		j = 1;
		
		for (vector<pair<string,float> >::const_iterator it = solution.second.begin(); it != solution.second.end(); ++it)
		{
			INFO("  " << it->first << ": ");
			ERR(it->second << endl);
			
			if ((it->first.at(0) == 'r') && (it->first.find("p") == string::npos) && (it->first.find("n") == string::npos))
			{
				gnuplotFile << i << " " << j << " " << it->second << endl;
				
				++j;
				
				if (j > gridColumns)
				{
					++i;
					j = 1;
				}
			}
		}
		
		INFO(endl << "Solution value/cost: ");
		ERR(solution.first << endl);
	}
}

void PLearner::generate(const string& filename) const
{
#ifdef ANALYZE_COMPUTATION_TIME
	Timestamp initial;
#endif
	
	WARN("Generating IRL problem..." << flush);
	
	learner.generate(filename,gridRows * gridColumns,numberOfActions,probabilityMatrices,*policy,gamma,lambda);
	
#ifdef ANALYZE_COMPUTATION_TIME
	WARN("done. (" << (Timestamp() - initial).getMs() << " ms)" << endl);
#else
	WARN("done." << endl);
#endif
}

Symbolic PLearner::generateProbabilityMatrix(Utils::GridAction action) const
{
	static int counter = 1;
	
	stringstream s;
	int rowDesiredState, columnDesiredState, rowState, columnState;
	int numberOfStates;
	
	numberOfStates = gridRows * gridColumns;
	
	s << "Pa_" << counter++;
	
	Symbolic probabilityMatrix(s.str(),numberOfStates,numberOfStates);
	
	for (int i = 0; i < numberOfStates; ++i)
	{
		for (int j = 0; j < numberOfStates; ++j)
		{
			probabilityMatrix(i,j) = 0;
		}
	}
	
	for (int i = 1; i <= numberOfStates; ++i)
	{
		rowState = ceil(i / (float) gridRows);
		columnState = i - ((rowState - 1) * gridRows);
		
		for (int j = 1; j <= numberOfStates; ++j)
		{
			rowDesiredState = ceil(j / (float) gridRows);
			columnDesiredState = j - ((rowDesiredState - 1) * gridRows);
			
			Utils::GridActionResult result = tryMove(rowState,columnState,rowDesiredState,columnDesiredState,action);
			
			/// The movement is feasible.
			if (result == Utils::Succeeded)
			{
				probabilityMatrix(numberOfStates - i,j - 1) = 1 - probabilityRandomAction;
			}
			else if (result == Utils::Impossible)
			{
				probabilityMatrix(numberOfStates - i,j - 1) = 0;
			}
			else if (result == Utils::OutsideGrid)
			{
				if (i == j)	probabilityMatrix(numberOfStates - i,j - 1) = 1 - probabilityRandomAction;
			}
			
			/// Actions start from 0 (see declaration in Utils.h).
			for (int k = 0; k < numberOfActions; ++k)
			{
				if (k == action) continue;
				
				result = tryMove(rowState,columnState,rowDesiredState,columnDesiredState,static_cast<Utils::GridAction>(k));
				
				if (result == Utils::Succeeded)
				{
					probabilityMatrix(numberOfStates - i,j - 1) += (probabilityRandomAction / (numberOfActions - 1));
				}
				else if (result == Utils::OutsideGrid)
				{
					if (i == j)
					{
						probabilityMatrix(numberOfStates - i,j - 1) += (probabilityRandomAction / (numberOfActions - 1));
					}
				}
			}
		}
	}
	
#ifdef PRINT_MATRICES
	INFO(endl << "Transition probability matrix: " << Utils::gridActionToName(action) << endl);
	
	for (int i = 0; i < numberOfStates; ++i)
	{
		for (int j = 0; j < numberOfStates; ++j)
		{
			DEBUG(probabilityMatrix(i,j) << " ");
		}
		
		INFO(endl);
	}
#endif
	
	return probabilityMatrix;
}

void PLearner::load(const string& filename)
{
	learner.load(filename);
}

Utils::GridActionResult PLearner::tryMove(int rowState, int columnState, int rowDesiredState, int columnDesiredState, Utils::GridAction action) const
{
	if (abs(rowState - rowDesiredState) + abs(columnState - columnDesiredState) > 1) return Utils::Impossible;
	
	if (action == Utils::East)
	{
		if (((columnState + 1) == columnDesiredState) && (rowState == rowDesiredState)) return Utils::Succeeded;
		else if ((columnState + 1) > gridColumns) return Utils::OutsideGrid;
	}
	
	if (action == Utils::North)
	{
		if (((rowState + 1) == rowDesiredState) && (columnState == columnDesiredState)) return Utils::Succeeded;
		else if ((rowState + 1) > gridRows) return Utils::OutsideGrid;
	}
	
	if (action == Utils::South)
	{
		if (((rowState - 1) == rowDesiredState) && (columnState == columnDesiredState)) return Utils::Succeeded;
		else if ((rowState - 1) <= 0) return Utils::OutsideGrid;
	}
	
	if (action == Utils::West)
	{
		if (((columnState - 1) == columnDesiredState) && (rowState == rowDesiredState)) return Utils::Succeeded;
		else if ((columnState - 1) <= 0) return Utils::OutsideGrid;
	}
	
	/// The state can be reached by using a diagonal movement.
	return Utils::Valid;
}
