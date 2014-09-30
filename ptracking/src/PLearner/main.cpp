#include "PLearner.h"
#include <Manfield/utils/debugutils.h>
#include <stdlib.h>

using namespace std;

int main(int argc, char** argv)
{
	if (argc != 3)
	{
		ERR("Usage: ./PLearner <generate | load> <problem-file>." << endl);
		
		exit(-1);
	}
	
	if (argv[1] != 0)
	{
		if ((string(argv[1]) != "generate") && (string(argv[1]) != "load"))
		{
			ERR("Usage: ./PLearner <generate | load> <problem-file>." << endl);
			
			exit(-1);
		}
	}
	
	PLearner pLearner;
	
	if (string(argv[1]) == "generate") pLearner.generate(argv[2]);
	
	pLearner.load(argv[2]);
	pLearner.exec();
	
	return 0;
}
