#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include "ground_removal.hpp"

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr readPCDFile(const std::string& filename)
{
	typename pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>);

	if(pcl::io::loadPCDFile<PointT>(filename, *result) == -1)
	{
		throw std::runtime_error("Unable to read file from '"+filename+"'");
	}

	return result;

}

void doBasicVisualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloud)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Basic Visualization"));
	viewer->setBackgroundColor(0,0,0);
	viewer->addPointCloud(pPointCloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	while(!viewer->wasStopped())
		viewer->spinOnce();

}

void doBinVisualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloud, unsigned int numBins)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pVisPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	std::unique_ptr<ugr::lidar::ground_removal::SegmentArray<pcl::PointXYZ>> pSegmentArray = ugr::lidar::ground_removal::assignPointsToBinsAndSegments<pcl::PointXYZ>(*pPointCloud, 1, numBins);

	typedef struct {unsigned int r,g,b;} Colour;

	std::array<Colour, 3> colours = {Colour{255,0,0},Colour{0,255,0},Colour{0,0,255}};

	unsigned int binIndex = 0;

	for(const ugr::lidar::ground_removal::PointArray<pcl::PointXYZ>& pointArray : (*pSegmentArray)[0])
	{
		for(const pcl::PointXYZ* pPoint : pointArray)
		{
			Colour colour = colours[binIndex%colours.size()];
			pcl::PointXYZRGB colouredPoint;
			colouredPoint.x = pPoint->x, colouredPoint.y = pPoint->y, colouredPoint.z = pPoint->z;
			colouredPoint.r = colour.r, colouredPoint.g = colour.g, colouredPoint.b = colour.b;
			pVisPointCloud->push_back(colouredPoint);
		}
		binIndex++;
	}

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Bin Visualization"));
	viewer->setBackgroundColor(0,0,0);
	viewer->addPointCloud(pVisPointCloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	while(!viewer->wasStopped())
		viewer->spinOnce();

}

void doSegmentVisualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloud, unsigned int numSegments)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pVisPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	std::unique_ptr<ugr::lidar::ground_removal::SegmentArray<pcl::PointXYZ>> pSegmentArray = ugr::lidar::ground_removal::assignPointsToBinsAndSegments<pcl::PointXYZ>(*pPointCloud, numSegments, 1);

	typedef struct {unsigned int r,g,b;} Colour;

	std::array<Colour, 3> colours = {Colour{255,0,0},Colour{0,255,0},Colour{0,0,255}};

	unsigned int segmentIndex = 0;

	for(const ugr::lidar::ground_removal::BinArray<pcl::PointXYZ>& binArray : (*pSegmentArray))
	{
		Colour colour = colours[segmentIndex%colours.size()];
		for(const ugr::lidar::ground_removal::PointArray<pcl::PointXYZ>& pointArray : binArray)
		{
		for(const pcl::PointXYZ* pPoint : pointArray)
			{
				pcl::PointXYZRGB colouredPoint;
				colouredPoint.x = pPoint->x, colouredPoint.y = pPoint->y, colouredPoint.z = pPoint->z;
				colouredPoint.r = colour.r, colouredPoint.g = colour.g, colouredPoint.b = colour.b;
				pVisPointCloud->push_back(colouredPoint);
			}
		}
		segmentIndex++;
	}

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Segment Visualization"));
	viewer->setBackgroundColor(0,0,0);
	viewer->addPointCloud(pVisPointCloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	while(!viewer->wasStopped())
		viewer->spinOnce();
}

void doGroundPlaneLineVisualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloud, const std::vector<std::vector<ugr::lidar::ground_removal::Line>>& lines)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Ground Plane Line Visualization"));	
	viewer->setBackgroundColor(0,0,0);
	viewer->addPointCloud(pPointCloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	unsigned int numSegments = lines.size();
	const float pi = std::atan(1)*4;

	for(unsigned int segmentIndex = 0; segmentIndex < numSegments; segmentIndex++)
	{
		float segmentAngle = ((pi*2)/numSegments)*segmentIndex + ((pi*2)/numSegments)/2;
		float lx = std::cos(segmentAngle);
		float ly = std::sin(segmentAngle);
		unsigned int j = 0;
		for(const ugr::lidar::ground_removal::Line& l : lines[segmentIndex])
		{
			pcl::PointXYZ begin(l.beginPoint.x*lx, l.beginPoint.x*ly, l.beginPoint.y);
			pcl::PointXYZ end(l.endPoint.x*lx, l.endPoint.x*ly, l.endPoint.y);
			viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(begin, end, "line_"+std::to_string(segmentIndex)+"_"+std::to_string(j));
			j++;
		}
	}

	while(!viewer->wasStopped())
		viewer->spinOnce();

}

void doBeforeAfterVisualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloudBefore, const pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloudAfter)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Before-After Visualization"));
	viewer->setBackgroundColor(0,0,0);
	viewer->addPointCloud(pPointCloudBefore, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();


	bool pointCloudBefore = true;

	unsigned int i = 0;

	while(!viewer->wasStopped())
	{
		
		using namespace std::chrono_literals;
		
		viewer->spinOnce(100);
		std::this_thread::sleep_for(100ms);
		i++;

		if(i == 10)
		{
			i = 0;
			viewer->removeAllPointClouds();
			if(pointCloudBefore)
			{
				viewer->addPointCloud(pPointCloudAfter, "cloud");
			}
			else
			{
				viewer->addPointCloud(pPointCloudBefore, "cloud");
			}
			pointCloudBefore = !pointCloudBefore;
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		}

	}	

}

int main(int argc, char* argv[])
{
	std::string dataFileName;

	if(argc < 2)
	{
		dataFileName = "sample_data/cloud1.pcd";
	}
	else
	{
		dataFileName = argv[1];
	}	

	unsigned int numSegments = 128, numBins = 256;
	std::string vis = "basic";

	if(argc >= 2)
	{
		vis = argv[2];
	}

	if(argc >= 5)
	{

		numSegments = std::stoul(argv[3]);
		numBins = std::stoul(argv[4]);
	}

	if(vis == "basic")
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud = readPCDFile<pcl::PointXYZ>(dataFileName);
		doBasicVisualization(pCloud);
	}
	else if(vis == "segments")
	{
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud = readPCDFile<pcl::PointXYZ>(dataFileName);
		doSegmentVisualization(pCloud, numSegments);
	}
	else if(vis == "bins")
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud = readPCDFile<pcl::PointXYZ>(dataFileName);
		doBinVisualization(pCloud, numBins);
	}
	else if(vis == "lines")
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud = readPCDFile<pcl::PointXYZ>(dataFileName);
		std::vector<std::vector<ugr::lidar::ground_removal::Line>> groundLines;
		std::unique_ptr<ugr::lidar::ground_removal::SegmentArray<pcl::PointXYZ>> pSegmentArray = ugr::lidar::ground_removal::assignPointsToBinsAndSegments(*pCloud, numSegments, numBins);
		for(const ugr::lidar::ground_removal::BinArray<pcl::PointXYZ>& binArray : *pSegmentArray)
		{
			groundLines.push_back(ugr::lidar::ground_removal::groundPlaneLinesForSegment<pcl::PointXYZ>(binArray));
		}
		doGroundPlaneLineVisualization(pCloud, groundLines);
	}
	else if(vis == "beforeafter")
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud = readPCDFile<pcl::PointXYZ>(dataFileName);
		std::unique_ptr<ugr::lidar::ground_removal::SegmentArray<pcl::PointXYZ>> pSegmentArray = ugr::lidar::ground_removal::assignPointsToBinsAndSegments<pcl::PointXYZ>(*pCloud, numSegments, numBins);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudNoGround = ugr::lidar::ground_removal::groundRemoval<pcl::PointXYZ>(*pSegmentArray);
		doBeforeAfterVisualization(pCloud, pCloudNoGround);
	}


	return 0;

}
