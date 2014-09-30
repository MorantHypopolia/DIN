#include "PVisualizer.h"
#include <Manfield/utils/debugutils.h>
#include <stdlib.h>

using namespace std;

int main(int argc, char** argv)
{
	if (argc != 3)
	{
		ERR("Usage: ./PVisualizer <dataset-image> <results-xml-file>." << endl);
		
		exit(-1);
	}
	
	PVisualizer pVisualizer;
	
	pVisualizer.exec(argv[1],argv[2]);
	
	return 0;
}
