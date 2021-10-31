#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>
#include <memory>
#include <cstdlib>
#include <algorithm>
#include <chrono>	
#include <thread>

// 2d array where each element is a list of points.
template<typename PointT>
using BinArray = std::vector< std::vector< std::vector< PointT* > > >;

// Expects only pcl point types e.g. PointXYZ, PointXYZRGB...
// Returns a bin assignment for each point as a vector containing elements of the form: ( (indexSegment, indexBin), PointT* )
// Experimental implementation, assumes a circular FOV of radius 2pi. Will result in many empty bins.
template<typename PointT>
std::unique_ptr< BinArray<PointT>  > assignBins(pcl::PointCloud<PointT>& pc, float viewDistance,
		unsigned int numBins, unsigned int numSegments)
{
	const float pi = 4.0f*std::atan(1);
	std::unique_ptr<BinArray<PointT>> binArray(new BinArray<PointT>(numSegments, std::vector<std::vector<PointT*>>(numBins)));

	float segmentAngleStep = 2.0f*pi / numSegments;
	float binRadiusStep = viewDistance / numBins;

	size_t i = 0;

	for(PointT& p : pc)
	{
		float x = p.x;
		float y = p.y;
		float angleWithPositiveXAxis = std::atan2(y,x);
		if (angleWithPositiveXAxis < 0)
			angleWithPositiveXAxis = 2*pi + angleWithPositiveXAxis;

		unsigned int segmentIndex = (unsigned int)(angleWithPositiveXAxis / segmentAngleStep);
		float distance = std::sqrt(x*x+y*y);

		unsigned int binIndex = std::min((unsigned int)(distance/binRadiusStep), numBins-1);		
		(*binArray)[segmentIndex][binIndex].push_back(&p);
	}

	return binArray;
	
}	

typedef std::pair<float, float> Point2D;

template<typename PointT>
Point2D getPrototype(std::vector<PointT*>& bin)
{
	PointT* p3d = *std::min_element(bin.begin(), bin.end(), [](PointT* p1, PointT* p2){ return p1->z < p2->z; });
	Point2D p2d = std::make_pair(std::sqrt(p3d->x*p3d->x+p3d->y*p3d->y), p3d->z);
	return p2d;
}

typedef std::pair<float, float> Line2D;

// Line of best fit using the 2D point information.
// least squares method, yoinked from https://web.archive.org/web/20150715022401/http://faculty.cs.niu.edu/~hutchins/csci230/best-fit.htm
// first=m, second=c
Line2D fitLine2D(std::vector< Point2D >& xyValues)
{
	float sumX = 0.0f;
	float sumY = 0.0f;
	float sumXX = 0.0f;
	float sumXY = 0.0f;

	for(std::pair<float, float> point : xyValues)
	{
		sumX += point.first;
		sumY += point.second;
		sumXX += point.first*point.first;
		sumXY += point.first*point.second;
	}

	float count = (float)xyValues.size();
	float xMean = sumX / count;
	float yMean = sumY / count;
	float slope = (sumXY - sumX * yMean) / (sumXX - sumX*xMean);
	float c = yMean - slope*xMean;

	return std::make_pair(slope, c);
}

float distancePointToLine(Point2D point, Line2D line)
{
	float x1 = point.first, x2 = point.first;
	float y1 = point.second, y2 = x2*line.first+line.second;
	return std::sqrt(std::pow(x1-x2,2)+std::pow(y1-y2,2));
}

float getFitRMSE(const Line2D& l, const std::vector<Point2D>& points)
{
	float sum = 0;
	float count = points.size();
	for(const Point2D& p : points)
	{
		float x = p.first;
		float y1 = l.first*x + l.second;
		float y2 = p.second;
		sum += std::pow(y1-y2, 2)/count;
	}
	float error = std::sqrt(sum);
	return error;
}

// Gets ground plane lines in a segment.
template<typename PointT>
std::vector<Line2D> getGroundPlaneLines(std::vector<std::vector<PointT*>>& array)
{
	float tSmall = -100.0f, tMax = 100.0f;
	float tYInt = 100.0f;
	float maxPointDistanceToLine = 100.0f;
	float tRMSE = 0.06f;

	std::vector<Line2D> lines;
	std::vector<Point2D> linePoints;

	for(size_t i = 0, c = 0; i < array.size(); i++)
	{
		std::vector<PointT*>& bin = array[i];
		if(bin.size() != 0)
		{
			Point2D prototypePoint = getPrototype(bin);
			if(linePoints.size() >= 2)
			{
				linePoints.push_back(prototypePoint);
				Line2D l = fitLine2D(linePoints);
				if (!( std::abs(l.first) < tMax && (l.first > tSmall || std::abs(l.second) <= tYInt) && getFitRMSE(l, linePoints) < tRMSE))
				{
					linePoints.pop_back();
					l = fitLine2D(linePoints);
					lines.push_back(l);
					linePoints = {};
					c++;
					i--;
				}
			}
			else
			{
				if(c == 0 || linePoints.size() == 0 || distancePointToLine(prototypePoint, lines[c-1]) <= maxPointDistanceToLine)
				{
					linePoints.push_back(prototypePoint);
				}
			}
		}
	}
	std::cout << std::endl;
	return lines;

}

int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("sample_data/cloud1.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read sample data.\n");
		return -1;
	}

	// find the furthest point from the sensor
	auto farthestPoint =  *std::max_element(cloud->begin(), cloud->end(), [](auto p1, auto p2){return p1.x*p1.x+p1.y*p1.y+p1.z*p1.z < p2.x*p2.x+p2.y*p2.y+p2.z*p2.z;});
	float viewDistance = std::sqrt(farthestPoint.x*farthestPoint.x+farthestPoint.y*farthestPoint.y+farthestPoint.z*farthestPoint.z);

	std::cout << "Furthest point: " << viewDistance << std::endl;

	const unsigned int numBins = 32, numSegments = 32;

	std::unique_ptr<BinArray<pcl::PointXYZRGB>> binArray = assignBins(*cloud, viewDistance, numBins, numSegments);

	size_t i = 0, j = 0;

	// colour each segment differently
	for (auto& segmentArray : *binArray)
	{
		float r = std::rand() % 256, g = std::rand() % 256, b = std::rand() % 256;
		for(auto& pointArray : segmentArray)
		{
			for (auto pPoint : pointArray)
			{
				pPoint->r = r, pPoint->g = g, pPoint->b = b;							
			}
			j++;
		}
		i++;
		j = 0;
	}

	for(size_t x = 0; x < binArray->size(); x++)
	{
		using namespace std;
		cout << "Segment: " << x+1 << std::endl;
		auto groundPlaneLines = getGroundPlaneLines((*binArray)[x]);
		for(auto l : groundPlaneLines)
		{
			
		}
	}

	i = 0, j = 0;

	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud);
	while(!viewer.wasStopped())
	{
		/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr fraction(new pcl::PointCloud<pcl::PointXYZRGB>);	

		auto iterator = binAssignment->begin();

		do
		{
			iterator = std::find_if(iterator, binAssignment->end(), [i,j](auto& assignment){ return assignment.first.first == i && assignment.first.second == j; });
			if (iterator != binAssignment->end())
			{
				fraction->push_back(*(iterator->second));	
				std::advance(iterator, 1);
			}
		} while(iterator != std::end(*binAssignment)); 
		std::cout << "Fraction size: " << fraction->size() << std::endl;
		j++;
		if(j == numBins)
		{
			j = 0;
			i++;
		}
		if(i == numSegments)
			i = 0;


		using namespace std::chrono_literals;
		
		if (fraction->size() != 0)	
		{
			std::this_thread::sleep_for(2000ms);
			viewer.showCloud(fraction);
		}*/
	}
		
	return 0;
}
