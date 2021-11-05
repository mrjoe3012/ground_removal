// OLD CODE FILE IGNORE

#include <iostream>
#include <cmath>
#include <vector>
#include <memory>
#include <cstdlib>
#include <algorithm>
#include <chrono>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

// essential data structures

// start of vector definitions

struct Vector2
{
	Vector2() : Vector2(0,0) {};
	Vector2(float x, float y) : x(x),y(y) {};
	Vector2(const Vector2& rhs) : x(rhs.x),y(rhs.y) {};

	friend Vector2 operator+(const Vector2& lhs, const Vector2& rhs);
	friend Vector2 operator-(const Vector2& lhs, const Vector2& rhs);
	friend Vector2 operator*(const Vector2& rhs, const float& lhs);
	friend Vector2 operator*(const float& rhs, const Vector2& lhs);
	friend Vector2 operator/(const Vector2& lhs, const float& rhs);
	friend bool operator==(const Vector2& lhs, const Vector2& rhs);

	// convert a 3d point to a 2d point based on its distance
	// from the centre of the 3d space and its height
	template<typename PointT>
	static Vector2 fromPclPoint(const PointT& p) { return Vector2(std::sqrt(std::pow(p.x,2) + std::pow(p.y,2)), p.z); };

	float sqrMagnitude() const { return std::pow(x,2) + std::pow(y,2); };
	float magnitude() const { return std::sqrt(sqrMagnitude()); };

	float x, y;
};

Vector2 operator+(const Vector2& lhs, const Vector2& rhs)
{
	return Vector2(lhs.x+rhs.x, lhs.y+rhs.y);
}

Vector2 operator-(const Vector2& lhs, const Vector2& rhs)
{
	return lhs + (-1*rhs);
}

Vector2 operator*(const Vector2& lhs, const float& rhs)
{
	return Vector2(lhs.x*rhs,lhs.y*rhs);
}

Vector2 operator*(const float& lhs, const Vector2& rhs)
{
	return operator*(rhs, lhs);
}

Vector2 operator/(const Vector2& lhs, const float& rhs)
{
	return (1.0f/rhs) * lhs;
}

bool operator==(const Vector2& lhs, const Vector2& rhs)
{
	return lhs.x==rhs.x && lhs.y==rhs.y;
}

// end of vector definitions

// start of line definitions

struct Line
{
	Line(float gradient, float yIntercept, Vector2 beginPoint, Vector2 endPoint) : gradient(gradient), yIntercept(yIntercept), beginPoint(beginPoint), endPoint(endPoint) {};
	Line(const Line& rhs) : Line(rhs.gradient, rhs.yIntercept, rhs.beginPoint, rhs.endPoint) {};

	float gradient, yIntercept;
	Vector2 beginPoint, endPoint;
};

// end of line definitions

// typedefs

// represents an array of pcl::Pointx pointers, used to represents points within a bin (within a segment)
template<typename PointT>
using PointArray = std::vector<const PointT*>;

// represents an array of bins, belonging to a specific segment
template<typename PointT>
using BinArray = std::vector< PointArray<PointT> >;

// the data structure we will be using to store point->(segment[bin]) assignments.
template<typename PointT>
using SegmentArray = std::vector< BinArray<PointT> >; 

// (smart) pointer to a point cloud
template<typename PointT>
using PCPtr = typename pcl::PointCloud<PointT>::Ptr;

// end of typedefs

// Currently always divides an FOV of 360degrees. This means that FOVs which are not 360degrees or 180degrees will have
// uneven segment assignments. Although given the use-case for the algorithm, it's unlikely that we would ever work with
// an FOV that is not 180degrees. Additionally, converting this function to work with any FOV should be relatively
// trivial.
// Note that we are templating this function (as well as many others) so that they work with a variety of pcl::Point data
// types e.g. pcl::PointXYZ, pcl::PointXYZRGB, ...
template<typename PointT>
std::unique_ptr< SegmentArray< PointT > > assignPointsToBinsAndSegments(const pcl::PointCloud<PointT>& cloud, int numberOfSegments, int numberOfBins)
{
	const float pi = 4*std::atan(1);

	// construct the result so that it already contains the correct number of arrays for segments and bins
	std::unique_ptr< SegmentArray< PointT > > pSegmentArray(new SegmentArray<PointT>(numberOfSegments, BinArray<PointT>(numberOfBins)));	

	// figure out how far away the farthest point is
	// so that we can determine how big each bin should
	// be.
	// Note that using the distance squared is acceptable here
	const PointT& farthestPoint = *std::max_element(cloud.begin(), cloud.end(), [](const PointT& p1, const PointT& p2){ return std::pow(p1.x, 2) + std::pow(p1.y, 2) < std::pow(p2.x, 2) + std::pow(p2.y, 2); });
	float farthestPointDistance = std::sqrt(std::pow(farthestPoint.x,2)+std::pow(farthestPoint.y,2));

	// distance between edge of each bin
	float binRange = farthestPointDistance / numberOfBins;
	// angle between each segment edge
	float segmentAngle = (2.0f*pi) / numberOfSegments;

	for(const PointT& point : cloud)
	{
		float pointDistance = std::sqrt(std::pow(point.x,2)+std::pow(point.y,2));
		float pointAngleWithXAxis = std::atan2(point.y, point.x);
		if(pointAngleWithXAxis < 0)
			pointAngleWithXAxis = 2.0f*pi + pointAngleWithXAxis;

		
		unsigned int segmentIndex = static_cast<unsigned int>(pointAngleWithXAxis/segmentAngle);
		unsigned int binIndex = std::min(static_cast<unsigned int>(pointDistance/binRange), static_cast<unsigned int>(numberOfBins-1));

		// assign the bin according to its index
		(*pSegmentArray)[segmentIndex][binIndex].push_back(&point);
	}


	return pSegmentArray;

}

// fits a line through an array of 2d points
// least squares method from https://web.archive.org/web/20150715022401/http://faculty.cs.niu.edu/~hutchins/csci230/best-fit.htm
Line fitLine2D(const std::vector<Vector2>& points)
{
	float sumX = 0.0f, sumY = 0.0f, sumXX = 0.0f, sumXY = 0.0f;

	for(const Vector2& point : points)
	{
		sumX += point.x;
		sumY += point.y;
		sumXX += point.x*point.x;
		sumXY += point.x*point.y;
	}

	float count = points.size();
	float xMean = sumX/count, yMean = sumY/count;
	float slope = (sumXY - sumX*yMean) / (sumXX - sumX*xMean);
	float yIntercept = yMean - slope*xMean;

	Vector2 startPoint(points.front().x, points.front().x*slope + yIntercept);
	Vector2 endPoint(points.back().x, points.back().x*slope+yIntercept);

	return Line(slope, yIntercept, startPoint, endPoint);
}

// returns the minimum distance between a (2d) point and a (2d) line.
float distanceFromPointToLine(const Vector2& point, const Line& line)
{
	Vector2 v1(point.x, point.y);
	Vector2 v2(point.x, point.x*line.gradient + line.yIntercept);
	return (v1-v2).magnitude();
}

// returns the root mean square error of a fit
// https://en.wikipedia.org/wiki/Root-mean-square_deviation
float fitRMSE(const Line& line, const std::vector<Vector2>& points)
{

	float sum = 0;
	float count = points.size();
	
	for(const Vector2& point : points)
	{
		float x = point.x;
		float y = point.y;
		float yExpected = line.gradient*x + line.yIntercept;
		sum += std::pow(yExpected-y, 2);
	}

	return std::sqrt(sum/count);

}

// Gets the prototype point, which is the point with lowest
// z value
template<typename PointT>
Vector2 prototypePoint(const PointArray<PointT>& pointArray)
{
	const PointT* pPoint = *std::min_element(pointArray.begin(), pointArray.end(), [](const PointT* lhs, const PointT* rhs){ return lhs->z < rhs->z; });
	// convert 3d point into 2d point
	Vector2 point2d = Vector2::fromPclPoint(*pPoint);
	return point2d;
}

// returns a series of ground plane line approximations for a specific segment
// (algorithm 1 in "Fast Segmentation of 3D Point Clouds for Gound Vehicles" pg.562)
template<typename PointT>
std::vector<Line> groundPlaneLinesForSegment(const BinArray<PointT>& binArray)
{
	const float tM = 1.0f, tMSmall = 0;
	const float tB = 1.0f;
	const float tRMSE = 0.005f;
	const float tDPrev = 0.1f;

	// points which are incrementally populated and
	// are ultimately used to fit ground plane lines
	std::vector<Vector2> linePoints;
	// the ground plane lines
	std::vector<Line> lines;

	unsigned int numberOfBins = binArray.size();

	for(unsigned int i = 0, c = 0; i < numberOfBins; i++)
	{
		const std::vector<const PointT*>& pointArray = binArray[i];

		if(pointArray.size() > 0)
		{
			Vector2 prototype = prototypePoint(pointArray);
			if(linePoints.size() >= 2)
			{
				linePoints.push_back(prototype);
				Line line = fitLine2D(linePoints);
				std::cout << line.gradient << ", " << line.yIntercept << ", " <<  fitRMSE(line, linePoints) << std::endl;
				if(!(std::abs(line.gradient) <= tM && (line.gradient > tMSmall || std::abs(line.yIntercept) <= tB) && fitRMSE(line, linePoints) <= tRMSE))
				{
					linePoints.pop_back();
					line = fitLine2D(linePoints);
					lines.push_back(line);
					linePoints.clear();
					c++;
					i--;
				}

			}
			else
			{
				if(linePoints.size() != 0 || c == 0 || distanceFromPointToLine(prototype, lines[c-1]) <= tDPrev)
				{
					linePoints.push_back(prototype);
				}
			}
		}

	}

	std::cout << lines.size() << " ground plane lines" << std::endl;

	return lines;

}

// returns a new point cloud with (hopefully) less ground points
// TODO create a structure for the parameter set and allow it to be customised
// per-call, rather than hard-coded as constants.
template<typename PointT>
PCPtr<PointT> groundRemoval(const SegmentArray<PointT>& segmentArray)
{

	const float tDGround = 0.1f;

	float numberOfSegments = segmentArray.size();

	PCPtr<PointT> result(new pcl::PointCloud<PointT>);

	unsigned int segmentIndex = 0;

	for(const BinArray<PointT>& binArray : segmentArray)
	{
		std::vector<Line> groundPlaneLines = groundPlaneLinesForSegment(binArray);
		for(const PointArray<PointT>& pointArray : binArray)
		{
			for(const PointT* point : pointArray)
			{
				Vector2 point2d = Vector2::fromPclPoint(*point);
				bool groundPoint = false;

				// find the closest line to the ground point
				const Line* closestLine = nullptr;
				float distanceToClosestLine = std::numeric_limits<float>::max();
				for (const Line& line : groundPlaneLines)
				{
					float distanceToStart = (line.beginPoint - point2d).sqrMagnitude();
					float distanceToEnd = (line.endPoint - point2d).sqrMagnitude();
					if(distanceToEnd < distanceToClosestLine || distanceToStart < distanceToClosestLine)
					{
						closestLine = &line;
						distanceToClosestLine = std::min(distanceToStart, distanceToEnd);
					}
				}

				// check the point's minimum distance to the closest line against
				// the ground point distance threshold to determine whether or not we should remove it
				if(closestLine != nullptr)
				{
					float distanceToLine = distanceFromPointToLine(point2d, *closestLine);
					if(distanceToLine <= tDGround)
						groundPoint = true;
				}

				// add the point to the new cloud only if it isn't a ground point
				if(!groundPoint)
					result->push_back(*point);
					
			}
		}
		segmentIndex++;
	}	

	return result;

}

void pointPickingCallback(const pcl::visualization::PointPickingEvent& event)
{
	float x,y,z;
	event.getPoint(x,y,z);
	std::cout << "Point picking event! (" << x << "," << y << "," << z << ")" << std::endl;
}

int main()
{


	// get point cloud from .pcd file

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);

	if(pcl::io::loadPCDFile<pcl::PointXYZRGB>("sample_data/cloud8.pcd", *cloud1) == -1)
	{
		PCL_ERROR("Couldn't read sample data.");
		return -1;
	}

	unsigned int numSegments = 128, numBins = 256;

	std::unique_ptr<SegmentArray<pcl::PointXYZRGB>> pSegmentArray = assignPointsToBinsAndSegments(*cloud1, numSegments, numBins);


	// colour each point according to its segment
	for(BinArray<pcl::PointXYZRGB>& binArray : *pSegmentArray)
	{
		float r = 50 + std::rand()%206;
		float g = 50 + std::rand()%206;
		float b = 50 + std::rand()%206;
		for(PointArray<pcl::PointXYZRGB>& pointArray : binArray)
		{
			for(const pcl::PointXYZRGB* pPoint : pointArray)
			{
				pcl::PointXYZRGB* p = const_cast<pcl::PointXYZRGB*>(pPoint);
				p->r = r;
				p->g = g;
				p->b = b;
				p->r=p->g=p->b=255;
			}	
		}
	}

	// get cloud with ground removed
	
	auto cloud2 = groundRemoval(*pSegmentArray);
	
	// visualize point clouds

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> whiteHandler(cloud2, 255, 255, 255);

	viewer->registerPointPickingCallback(pointPickingCallback);

	viewer->setBackgroundColor(0,0,0);
	
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud1, whiteHandler, "cloud1");

	for(unsigned int i = 0; i < numSegments; i++)
	{
		const float pi = 4*std::atan(1);
		std::vector<Line> groundPlaneLines = groundPlaneLinesForSegment((*pSegmentArray)[i]);
		float segmentLineAngle = ((2*pi)/numSegments)*i + ((2*pi)/numSegments)/2;
		std::cout << "angle: " << segmentLineAngle << std::endl;
		float lx = std::cos(segmentLineAngle);
		float ly = std::sin(segmentLineAngle);
		unsigned int j = 0;
		for(const Line& l : groundPlaneLines)
		{
			pcl::PointXYZ begin(l.beginPoint.x*lx, l.beginPoint.x*ly, l.beginPoint.y);
			pcl::PointXYZ end(l.endPoint.x*lx, l.endPoint.x*ly, l.endPoint.y);
			//uncomment the line below
			//viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(begin, end, "line_"+std::to_string(i)+"_"+std::to_string(j));
			j++;
		}	

	}

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	unsigned int j = 0;
	bool pc1 = true;

	while(!viewer->wasStopped())
	{
		using namespace std::chrono_literals;
		viewer->spinOnce(100);
		std::this_thread::sleep_for(100ms);
		j++;
		if(j == 10)
		{
			// switch the point cloud every 1 second
			viewer->removePointCloud((pc1?"cloud1":"cloud2"));
			viewer->addPointCloud((pc1?cloud1:cloud2), (pc1?"cloud2":"cloud1"));
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, (pc1?"cloud2":"cloud1"));
			pc1=!pc1;
			j =  0;
		}
	}

	return 0;
}
