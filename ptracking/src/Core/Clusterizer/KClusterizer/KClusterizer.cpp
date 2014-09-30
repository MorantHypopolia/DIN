#include "KClusterizer.h"
#include <Utils/Utils.h>
#include <float.h>

using namespace std;

namespace PTracking
{
	KClusterizer::KClusterizer(int maxClusters) : kpoints(maxClusters), maxClusters(maxClusters) {;}
	
	KClusterizer::~KClusterizer() {;}
	
	void KClusterizer::clusterize(const PoseParticleVector& particleVector, float qualityThreshold)
	{
		multimap<int,PoseParticleVector> outliersMapping;
		map<float,Cluster> cluster;
		vector<pair<PoseParticleVector,Point2of> > clustersToBeAdded;
		vector<Point2of> allCentroids;
		vector<int> outliers;
		PoseParticleVector particles;
		float distance, temp;
		int closeClusterNumber, clusterIndex, i, j, kint, size;
		
		clusters.clear();
		kpoints.reset();
		
		for (PoseParticleVector::const_iterator it = particleVector.begin(); it != particleVector.end(); it++)
		{
			if (isFarFromAll(*it,qualityThreshold)) kpoints.push(*it);
		}
		
		kint = 0;
		
		for (PriorityBuffer<PoseParticle>::const_iterator cit = kpoints.begin(); cit != kpoints.end(); ++cit)
		{
			float index = (cit->second.pose.pose.x * 100000.0) + (cit->second.pose.pose.y * 50000.0) + cit->second.pose.pose.theta;
			
			Cluster c;
			
			c.k = cit->second;
			cluster[index] = c;
		}
		
		i = 0;
		
		for (PoseParticleVector::const_iterator it = particleVector.begin(); it != particleVector.end(); ++it, ++i)
		{
			int count;
			bool found;
			
			count = 0;
			found = false;
			
			for (PriorityBuffer<PoseParticle>::const_iterator pit = kpoints.begin(); (!found) && (pit != kpoints.end()); ++pit)
			{
				if (!isFarFrom(pit->second,*it,qualityThreshold))
				{
					float index = (pit->second.pose.pose.x * 100000.0) + (pit->second.pose.pose.y * 50000.0) + pit->second.pose.pose.theta;
					
					cluster[index].indices.push_back(i);
					found = true;
				}
				
				count++;
			}
			
			if (!found) outliers.push_back(i);
		}
		
		kint = 0;
		
		for (map<float,Cluster>::iterator it = cluster.begin(); it != cluster.end(); ++it, ++kint)
		{
			float count, cSum, rSum, sSum;
			
			rSum = 0.0;
			cSum = 0.0;
			sSum = 0.0;
			count = 0.0;
			
			particles.clear();
			
			for (vector<int>::const_iterator vit = it->second.indices.begin(); vit != it->second.indices.end(); ++vit, ++count)
			{
				const PoseParticle& p = particleVector[(*vit)];
				
				rSum += sqrt((p.pose.pose.x * p.pose.pose.x) + (p.pose.pose.y * p.pose.pose.y));
				cSum += cos(p.pose.pose.theta);
				sSum += sin(p.pose.pose.theta);
				
				particles.push_back(p);
			}
			
			if (particles.size() > 0)
			{
				clusters.push_back(make_pair(particles,Utils::calculateCentroid(particles)));
			}
		}
		
		for (vector<pair<PoseParticleVector,Point2of> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
		{
			allCentroids.push_back(it->second);
		}
		
		i = 0;
		size = outliers.size();
		
		while (i < size)
		{
			const PoseParticle& outlier = particleVector.at(outliers.at(i));
			
			clusterIndex = 0;
			j = 0;
			distance = FLT_MAX;
			
			for (vector<Point2of>::iterator it = allCentroids.begin(); it != allCentroids.end(); ++it)
			{
				temp = ((it->x - outlier.pose.pose.x) * (it->x - outlier.pose.pose.x)) + ((it->y - outlier.pose.pose.y) * (it->y - outlier.pose.pose.y));
				
				if (temp < distance)
				{
					clusterIndex = j;
					distance = temp;
				}
				
				++j;
			}
			
			++i;
			
			const multimap<int,PoseParticleVector>::iterator& outlierMapping = outliersMapping.find(clusterIndex);
			
			if (outlierMapping == outliersMapping.end())
			{
				PoseParticleVector v;
				
				v.push_back(outlier);
				
				outliersMapping.insert(make_pair(clusterIndex,v));
			}
			else outlierMapping->second.push_back(outlier);
		}
		
		for (multimap<int,PoseParticleVector>::iterator it = outliersMapping.begin(); it != outliersMapping.end(); ++it)
		{
			if (it->second.size() >= MIN_SIZE_CLUSTER_AFTER_SPLITTING)
			{
				clusters.push_back(make_pair(it->second,Utils::calculateCentroid(it->second)));
			}
		}
		
		i = 0;
		
		/// Analyzing whether a cluster is non Gaussian. If so, it will splitted into 2 clusters.
		for (vector<pair<PoseParticleVector,Point2of> >::iterator it = clusters.begin(); it != clusters.end(); ++i)
		{
			PoseParticle p1, p2;
			
			distance = FLT_MIN;
			
			for (PoseParticleVector::const_iterator it2 = it->first.begin(); it2 != it->first.end(); ++it2)
			{
				for (PoseParticleVector::const_iterator it3 = it->first.begin(); it3 != it->first.end(); ++it3)
				{
					temp = sqrt(((it2->pose.pose.x - it3->pose.pose.x) * (it2->pose.pose.x - it3->pose.pose.x)) + ((it2->pose.pose.y - it3->pose.pose.y) * (it2->pose.pose.y - it3->pose.pose.y)));
					
					if (temp > distance)
					{
						p1 = *it2;
						p2 = *it3;
						distance = temp;
					}
				}
			}
			
			if (distance > qualityThreshold)
			{
				PoseParticleVector part1, part2;
				float temp2;
				
				for (PoseParticleVector::const_iterator it2 = it->first.begin(); it2 != it->first.end(); ++it2)
				{
					temp = sqrt(((it2->pose.pose.x - p1.pose.pose.x) * (it2->pose.pose.x - p1.pose.pose.x)) + ((it2->pose.pose.y - p1.pose.pose.y) * (it2->pose.pose.y - p1.pose.pose.y)));
					temp2 = sqrt(((it2->pose.pose.x - p2.pose.pose.x) * (it2->pose.pose.x - p2.pose.pose.x)) + ((it2->pose.pose.y - p2.pose.pose.y) * (it2->pose.pose.y - p2.pose.pose.y)));
					
					if ((temp < temp2) && (temp < (0.5 * qualityThreshold))) part1.push_back(*it2);
					else if (temp2 < (0.5 * qualityThreshold)) part2.push_back(*it2);
				}
				
				if (part1.size() >= MIN_SIZE_CLUSTER_AFTER_SPLITTING) clustersToBeAdded.push_back(make_pair(part1,Utils::calculateCentroid(part1)));
				if (part2.size() >= MIN_SIZE_CLUSTER_AFTER_SPLITTING) clustersToBeAdded.push_back(make_pair(part2,Utils::calculateCentroid(part2)));
				
				it = clusters.erase(it);
			}
			else ++it;
		}
		
		for (vector<pair<PoseParticleVector,Point2of> >::const_iterator it = clustersToBeAdded.begin(); it != clustersToBeAdded.end(); ++it)
		{
			clusters.push_back(*it);
		}
		
		for (vector<pair<PoseParticleVector,Point2of> >::iterator it = clusters.begin(); it != clusters.end(); )
		{
			closeClusterNumber = 0;
			
			for (vector<pair<PoseParticleVector,Point2of> >::iterator it2 = clusters.begin(); it2 != clusters.end(); ++it2)
			{
				if (Utils::isTargetNear(it->second,it2->second,0.3 * qualityThreshold)) ++closeClusterNumber;
			}
			
			if (closeClusterNumber > 1) it = clusters.erase(it);
			else ++it;
		}
	}
	
	bool KClusterizer::isFarFrom(const PoseParticle& p1, const PoseParticle& p2, float qualityThreshold) const
	{
		return (sqrt(((p1.pose.pose.x - p2.pose.pose.x) * (p1.pose.pose.x - p2.pose.pose.x)) +
					 ((p1.pose.pose.y - p2.pose.pose.y) * (p1.pose.pose.y - p2.pose.pose.y))) > qualityThreshold);
	}
	
	bool KClusterizer::isFarFromAll(const PoseParticle& p, float qualityThreshold) const
	{
		float distance;
		bool isFarEnough;
		
		distance = qualityThreshold;
		
		isFarEnough = true;
		
		for (PriorityBuffer<PoseParticle>::const_iterator it = kpoints.begin(); (isFarEnough) && (it != kpoints.end()); ++it)
		{
			isFarEnough = isFarFrom(p,it->second,distance);
		}
		
		return isFarEnough;
	}
	
	void KClusterizer::setMaxClusterNumber(int k)
	{
		maxClusters = k;
		
		kpoints = PriorityBuffer<PoseParticle>(maxClusters);
	}
}
