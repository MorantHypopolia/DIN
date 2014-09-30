#include "PTracker.h"

int main(int argc, char** argv)
{
	if ((argc != 2) && (argc != 3))
	{
		ERR("Usage: ./PTracker <observation-file> [ <frame-rate> ]." << endl);
		
		exit(-1);
	}
	
	PTracker pTracker;
	int frameRate;
	
	if (argv[2] != 0) frameRate = atoi(argv[2]);
	else frameRate = 30;
	
	pTracker.exec(argv[1],frameRate);
	
	return 0;
}
