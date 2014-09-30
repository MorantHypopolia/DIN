#include "IRLearner.h"
#include <Manfield/utils/debugutils.h>
#include <ThirdParty/SymbolicC++/symbolicc++.h>
#include <CGAL/QP_functions.h>
#include <boost/tokenizer.hpp>
#include <stdlib.h>
#include <cfloat>
#include <fstream>
#include <iomanip>
#include <sstream>

typedef CGAL::MP_Float ET;
typedef CGAL::Quadratic_program_solution<ET> Solution;

using namespace std;
using namespace boost;
using namespace CGAL;

namespace PTracking
{
	IRLearner::IRLearner() : problem(0) {;}
	
	IRLearner::~IRLearner()
	{
		if (problem != 0) delete problem;
	}
	
	void IRLearner::generate(const string& filename, int numberOfStates, int numberOfActions, const vector<Symbolic>& probabilityMatrices, const Symbolic& policy, float gamma, float lambda) const
	{
		vector<pair<string,pair<string,float> > > bounds, columns;
		vector<pair<string,float> > rhs;
		vector<pair<char,string> > rows;
		Symbolic identity("I",numberOfStates,numberOfStates);
		Symbolic rewardMatrix("RewardMatrix",numberOfStates,1);
		Symbolic t("(I - gamma Pa_*) * R",numberOfStates,1);
		stringstream s;
		int counter, numberOfActionsAndStates;
		char constraintBaseName = 'c', newVariableBaseName = 'x',rewardVariableBaseName = 'r';
		
		for (int i = 0; i < numberOfStates; ++i)
		{
			s.str("");
			s.clear();
			
			s << rewardVariableBaseName << (i + 1);
			
			rewardMatrix(i,0) = s.str();
		}
		
		for (int i = 0; i < numberOfStates; ++i)
		{
			for (int j = 0; j < numberOfStates; ++j)
			{
				if (i == j)	identity(i,j) = 1;
				else identity(i,j) = 0;
			}
		}
		
		t = (identity - ((double) gamma * policy)).inverse() * rewardMatrix;
		
		/// Objective function.
		rows.push_back(make_pair('N',"obj"));
		
		/// Constraints for expressing the 1-norm of the objective function.
		for (counter = 1; counter <= numberOfStates; ++counter)
		{
			s.str("");
			s.clear();
			
			s << constraintBaseName << counter;
			
			rows.push_back(make_pair('E',s.str()));
		}
		
		numberOfActionsAndStates = numberOfActions * numberOfStates;
		
		/// Constraints of the problem.
		for (; counter <= (numberOfStates + numberOfActionsAndStates); ++counter)
		{
			s.str("");
			s.clear();
			
			s << constraintBaseName << counter;
			
			rows.push_back(make_pair('L',s.str()));
		}
		
		/// Columns for the objective function.
		for (counter = 1; counter <= numberOfStates; ++counter)
		{
			s.str("");
			s.clear();
			
			s << newVariableBaseName << counter;
			
			columns.push_back(make_pair(s.str(),make_pair("obj",-1)));
		}
		
		/// Columns for the (positive) reward variables introduced to express the 1-norm of the objective function.
		for (counter = 1; counter <= numberOfStates; ++counter)
		{
			s.str("");
			s.clear();
			
			s << rewardVariableBaseName << counter << "p";
			
			columns.push_back(make_pair(s.str(),make_pair("obj",lambda)));
		}
		
		/// Columns for the (negative) reward variables introduced to express the 1-norm of the objective function.
		for (counter = 1; counter <= numberOfStates; ++counter)
		{
			s.str("");
			s.clear();
			
			s << rewardVariableBaseName << counter << "n";
			
			columns.push_back(make_pair(s.str(),make_pair("obj",-lambda)));
		}
		
		Symbolic differenceProbabilityMatrices("Pa_*(i) - Pa_w(i)",1,numberOfStates);
		char_separator<char> starSeparator("*"), negativeSeparator("-","-",boost::keep_empty_tokens), positiveSeparator("+");
		stringstream constraintStream, constraintStreamCounter;
		string constraint, temp, variable;
		float value;
		int counter2;
		
		counter2 = 1;
		
		for (int i = 1; i <= numberOfStates; ++i, ++counter2)
		{
			constraintStreamCounter.str("");
			constraintStreamCounter.clear();
			
			constraintStreamCounter << constraintBaseName << counter2;
			
			s.str("");
			s.clear();
			
			s << rewardVariableBaseName << i;
			
			columns.push_back(make_pair(s.str(),make_pair(constraintStreamCounter.str(),1)));
			
			s.str("");
			s.clear();
			
			s << rewardVariableBaseName << i << "p";
			
			columns.push_back(make_pair(s.str(),make_pair(constraintStreamCounter.str(),-1)));
			
			s.str("");
			s.clear();
			
			s << rewardVariableBaseName << i << "n";
			
			columns.push_back(make_pair(s.str(),make_pair(constraintStreamCounter.str(),1)));
		}
		
		/// Because the starting state is in position bottom-left of the grid, while the matrices are written following the standard format.
		for (int i = (numberOfStates - 1), counter = 1; i >= 0; --i, ++counter)
		{
			for (int j = 0; j < numberOfActions; ++j, ++counter2)
			{
				constraintStreamCounter.str("");
				constraintStreamCounter.clear();
				
				constraintStreamCounter << constraintBaseName << counter2;
				
				for (int w = 0; w < numberOfStates; ++w)
				{
					differenceProbabilityMatrices(0,w) = (policy(i,w) - probabilityMatrices.at(j)(i,w)) * -1;
				}
				
				constraintStream.str("");
				constraintStream.clear();
				
				constraintStream << newVariableBaseName << counter << "+" << fixed;
				
				(differenceProbabilityMatrices * t).print(constraintStream);
				
				constraint = constraintStream.str();
				
				tokenizer<char_separator<char> > positiveTokens(constraint,positiveSeparator);
				
				for (tokenizer<char_separator<char> >::const_iterator it = positiveTokens.begin(); it != positiveTokens.end(); ++it)
				{
					if (it->size() == 0) continue;
					
					/// We have a constraint like: "x1+0 < 0". So, we can skip this token.
					if (*it == "0") continue;
					
					/// Multiple variables are present in the string.
					if ((count(it->begin(),it->end(),'*') > 1) || (it->find("-") != string::npos))
					{
						tokenizer<char_separator<char> > negativeTokens(*it,negativeSeparator);
						
						for (tokenizer<char_separator<char> >::const_iterator it2 = negativeTokens.begin(); it2 != negativeTokens.end(); ++it2)
						{
							if (it2->size() == 0) continue;
							
							temp = *it2;
							
							/// Keeping the '-' delimiter.
							if ((it2->size() == 1) && (*it2 == "-"))
							{
								++it2;
								temp += *it2;
							}
							
							tokenizer<char_separator<char> > starTokens(temp,starSeparator);
							tokenizer<char_separator<char> > starTokensCopy = starTokens;
							
							if (++starTokensCopy.begin() == starTokensCopy.end())
							{
								if (string(*starTokens.begin()).find("-") == string::npos)
								{
									variable = *starTokens.begin();
									value = 1;
								}
								else
								{
									variable = string(*starTokens.begin()).substr(1,string(*starTokens.begin()).size());
									value = -1;
								}
							}
							else
							{
								variable = *(++starTokens.begin());
								value = atof(string(*starTokens.begin()).c_str());
							}
							
							columns.push_back(make_pair(variable,make_pair(constraintStreamCounter.str(),value)));
						}
					}
					else
					{
						tokenizer<char_separator<char> > starTokens(*it,starSeparator);
						tokenizer<char_separator<char> > starTokensCopy = starTokens;
						
						if (++starTokensCopy.begin() == starTokensCopy.end())
						{
							if (string(*starTokens.begin()).find("-") == string::npos)
							{
								variable = *starTokens.begin();
								value = 1;
							}
							else
							{
								variable = string(*starTokens.begin()).substr(1,string(*starTokens.begin()).size());
								value = -1;
							}
						}
						else
						{
							variable = *(++starTokens.begin());
							value = atof(string(*starTokens.begin()).c_str());
						}
						
						columns.push_back(make_pair(variable,make_pair(constraintStreamCounter.str(),value)));
					}
				}
			}
		}
		
		/// Right-hand side of the objective function.
		rhs.push_back(make_pair("obj",0));
		
		/// Right-hand side of the constraints.
		for (counter = 1; counter <= (numberOfActionsAndStates + numberOfStates); ++counter)
		{
			s.str("");
			s.clear();
			
			s << constraintBaseName << counter;
			
			rhs.push_back(make_pair(s.str(),0));
		}
		
		/// Lower bounds of the reward variables.
		for (counter = 1; counter <= numberOfStates; ++counter)
		{
			s.str("");
			s.clear();
			
			s << rewardVariableBaseName << counter;
			
			bounds.push_back(make_pair("LO",make_pair(s.str(),-1)));
		}
		
		/// Upper bounds of the reward variables.
		for (counter = 1; counter <= numberOfStates; ++counter)
		{
			s.str("");
			s.clear();
			
			s << rewardVariableBaseName << counter;
			
			bounds.push_back(make_pair("UP",make_pair(s.str(),1)));
		}
		
		write(filename,rows,columns,rhs,bounds);
	}
	
	void IRLearner::load(const string& filename)
	{
		try
		{
			ifstream problemFile(filename.c_str());
			
			problem = new Program(problemFile);
			
			if (!problem->is_valid())
			{
				WARN("The problem is not valid. Exiting..." << endl);
				
				exit(-1);
			}
			
			if (!problem->is_linear())
			{
				WARN("The problem is not linear, therefore it cannot be resolved. Exiting..." << endl);
				
				exit(-1);
			}
		}
		catch (...)
		{
			ERR("An error occured when loadind the LP problem file '");
			INFO(filename);
			ERR("'. Exiting..." << endl);
			
			exit(-1);
		}
	}
	
	void IRLearner::solve()
	{
		try
		{
			/// Initializing the solution.
			solution.first = -FLT_MAX;
			solution.second = vector<pair<string,float> >();
			
			Solution s = solve_linear_program(*problem,ET());
			
			if (!s.is_valid())
			{
				ERR("Problem formulation is incorrect." << endl);
				
				return;
			}
			
			if (!s.is_infeasible())
			{
				if (!s.is_unbounded())
				{
					int j;
					
					solution.first = CGAL::to_double(s.objective_value());
					
					j = 0;
					
					for (Quadratic_program_solution<ET>::Variable_value_iterator it = s.variable_values_begin(); it < s.variable_values_end(); ++it, ++j)
					{
						solution.second.push_back(make_pair(problem->variable_name_by_index(j),CGAL::to_double(*it)));
					}
				}
				else
				{
					WARN("Problem is unlimited." << endl);
				}
			}
			else
			{
				WARN("Problem is overconstrained." << endl);
			}
		}
		catch (...)
		{
			ERR(endl << "An exception has been caught: unknown failure occured." << endl);
			
			exit(-1);
		}
	}
	
	void IRLearner::write(const string& filename, const vector<pair<char,string> >& rows, const vector<pair<string,pair<string,float> > >& columns, const vector<pair<string,float> >& rhs,
						  const vector<pair<string,pair<string,float> > >& bounds) const
	{
		ofstream file;
		unsigned int maxLength, maxLength2;
		
		file.open(filename.c_str());
		
		file << "NAME "	<< filename << endl << endl;
		
		file << "ROWS" << endl;
		
		for (vector<pair<char,string> >::const_iterator it = rows.begin(); it != rows.end(); ++it)
		{
			file << "  " << it->first << "  " << it->second << endl;
		}
		
		file << endl << "COLUMNS" << endl;
		
		maxLength = 0;
		
		for (vector<pair<string,pair<string,float> > >::const_iterator it = columns.begin(); it != columns.end(); ++it)
		{
			if (it->first.size() > maxLength) maxLength = it->first.size();
		}
		
		maxLength2 = 0;
		
		for (vector<pair<string,pair<string,float> > >::const_iterator it = columns.begin(); it != columns.end(); ++it)
		{
			if (it->second.first.size() > maxLength2) maxLength2 = it->second.first.size();
		}
		
		for (vector<pair<string,pair<string,float> > >::const_iterator it = columns.begin(); it != columns.end(); ++it)
		{
			file << "  " << it->first;
			
			for (unsigned int j = 1; j <= (2 + maxLength - it->first.size()); ++j) file << " ";
			
			file << it->second.first;
			
			for (unsigned int j = 1; j <= (2 + maxLength2 - it->second.first.size()); ++j) file << " ";
			
			if (it->second.second >= 0) file << " ";
			
			/// To avoid -0.000.
			if (it->second.second == 0) file << fixed << setprecision(3) << fabs(it->second.second) << endl;
			else file << fixed << setprecision(3) << it->second.second << endl;
		}
		
		file << endl << "RHS" << endl;
		
		maxLength = 0;
		
		for (vector<pair<string,float> >::const_iterator it = rhs.begin(); it != rhs.end(); ++it)
		{
			if (it->first.size() > maxLength) maxLength = it->first.size();
		}
		
		for (vector<pair<string,float> >::const_iterator it = rhs.begin(); it != rhs.end(); ++it)
		{
			file << "  rhs  " << it->first;
			
			for (unsigned int j = 1; j <= (2 + maxLength - it->first.size()); ++j) file << " ";
			
			if (it->second >= 0) file << " ";
			
			if (it->second == 0) file << fixed << setprecision(3) << fabs(it->second) << endl;
			else file << fixed << setprecision(3) << it->second << endl;
		}
		
		file << endl << "BOUNDS" << endl;
		
		maxLength = 0;
		
		for (vector<pair<string,pair<string,float> > >::const_iterator it = bounds.begin(); it != bounds.end(); ++it)
		{
			if (it->second.first.size() > maxLength) maxLength = it->second.first.size();
		}
		
		for (vector<pair<string,pair<string,float> > >::const_iterator it = bounds.begin(); it != bounds.end(); ++it)
		{
			file << "  " << it->first << "  BND  " << it->second.first;
			
			for (unsigned int j = 1; j <= (2 + maxLength - it->second.first.size()); ++j) file << " ";
			
			if (it->second.second >= 0) file << " ";
			
			if (it->second.second == 0) file << fixed << setprecision(3) << fabs(it->second.second) << endl;
			else file << fixed << setprecision(3) << it->second.second << endl;
		}
		
		file << "ENDATA" << endl;
		file.close();
	}
}
