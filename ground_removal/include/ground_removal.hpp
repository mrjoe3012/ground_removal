//===-- ugrdv_per...ion_lidar/include/ground_removal.hpp - Ground Removal Functions -*- C++ -*-===//
//
// Part of the UGRDV Project, under the UGRACING DRIVERLESS PRIVATE LICENSE.
// See the attached LICENSE.txt file in this projects upper most directory for
// license information.
//
// Copyright Joseph Agrane © 2021-2022
//
//===------------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains the  ground removal functions which come after filtering.
///
//===-------------------------------------------------------------------------------------------===//

#ifndef GROUND_REMOVAL_HPP_
#define GROUND_REMOVAL_HPP_

#include <pcl/point_cloud.h>

#include <vector>
#include <memory>

namespace ugr
{

namespace lidar
{

namespace ground_removal
{

class Vector2
{
public:
  Vector2();
  Vector2(float x, float y);

  friend Vector2 operator+(const Vector2 & lhs, const Vector2 & rhs);
  friend Vector2 operator-(const Vector2 & lhs, const Vector2 & rhs);
  friend Vector2 operator*(const Vector2 & rhs, const float & lhs);
  friend Vector2 operator*(const float & rhs, const Vector2 & lhs);
  friend Vector2 operator/(const Vector2 & lhs, const float & rhs);
  friend bool operator==(const Vector2 & lhs, const Vector2 & rhs);

  // convert a 3d point to a 2d point based on its distance
  // from the centre of the 3d space and its height
  template<typename PointT>
  static Vector2 fromPclPoint(const PointT & p);

  float sqrMagnitude() const;
  float magnitude() const;

  float x, y;
};

struct Line
{
  Line(float gradient, float yIntercept, Vector2 beginPoint, Vector2 endPoint);

  float gradient, yIntercept;
  Vector2 beginPoint, endPoint;
};

// Algorithm Parameters;
// Parameters used in the ground removal algorithm.
// tM: The maximum gradient a ground plane line can have.
// tMSmall: The gradient threshold for a 'small' gradient.
// tB: The maximum y intercept allowed for a line with a 'small' gradient.
// tDPrev: The maximum distance between the first point of a ground line and
// the previous ground line.
// tRMSE: The maximum allowed Root Mean Square Error for a line of best fit.

struct AlgorithmParameters
{
  float tM, tMSmall, tB, tRMSE, tDPrev, tDGround;
};

// aliases:

// represents an array of pcl::Pointx pointers,
// used to represents points within a bin (within a segment)
template<typename PointT>
using PointArray = std::vector<const PointT *>;

// represents an array of bins, belonging to a specific segment
template<typename PointT>
using BinArray = std::vector<PointArray<PointT>>;

// the data structure we will be using to store point->(segment[bin]) assignments.
template<typename PointT>
using SegmentArray = std::vector<BinArray<PointT>>;

// (shared) pointer to a point cloud
template<typename PointT>
using PCPtr = typename pcl::PointCloud<PointT>::Ptr;

// constants

const AlgorithmParameters DEFAULT_ALGORITHM_PARAMETERS =
{
  0.5,      // tM
  0.5f,     // tMSmall
  0.5f,     // tB
  0.005f,   // tRMSE
  0.25f,    // tDPrev
  0.225f,   // tDGround
};

const int
  DEFAULT_NUM_SEGMENTS = 10,
  DEFAULT_NUM_BINS = 6;

// ALGORITHM IMPLEMENTATION:

// Currently always divides an FOV of 360degrees.
// This means that FOVs which are not 360degrees or 180degrees will have
// uneven segment assignments. Although given the use-case for the algorithm,
//  it's unlikely that we would ever work with
// an FOV that is not 180degrees. Additionally,
// converting this function to work with any FOV should be relatively
// trivial.
// Note that we are templating this function (as well as many others)
// so that they work with a variety of pcl::Point data
// types e.g. pcl::PointXYZ, pcl::PointXYZRGB, ...
template<typename PointT>
std::unique_ptr<SegmentArray<PointT>> assignPointsToBinsAndSegments(
  const pcl::PointCloud<PointT> & cloud, int numberOfSegments = DEFAULT_NUM_SEGMENTS,
  int numberOfBins = DEFAULT_NUM_BINS);

// fits a line through an array of 2d points
// least squares method from https://web.archive.org/web/20150715022401/http://faculty.cs.niu.edu/~hutchins/csci230/best-fit.htm
Line fitLine2D(const std::vector<Vector2> & points);

// returns the minimum distance between a (2d) point and a (2d) line.
float distanceFromPointToLine(const Vector2 & point, const Line & line);

// returns the root mean square error of a fit
// https://en.wikipedia.org/wiki/Root-mean-square_deviation
float fitRMSE(const Line & line, const std::vector<Vector2> & points);

// Gets the prototype point, which is the point with lowest
// z value
template<typename PointT>
Vector2 prototypePoint(const PointArray<PointT> & pointArray);

// returns a series of ground plane line approximations for a specific segment
// (algorithm 1 in "Fast Segmentation of 3D Point Clouds for Gound Vehicles" pg.562)
template<typename PointT>
std::vector<Line> groundPlaneLinesForSegment(
  const BinArray<PointT> & binArray,
  const AlgorithmParameters & params = DEFAULT_ALGORITHM_PARAMETERS);

// performs ground removal and returns the filtered point cloud
template<typename PointT>
PCPtr<PointT> groundRemoval(
  const SegmentArray<PointT> & segmentArray,
  const AlgorithmParameters & params = DEFAULT_ALGORITHM_PARAMETERS);

// performs ground removal, returns the filtered point cloud and
// places all removed points in a seperate point cloud
template<typename PointT>
PCPtr<PointT> groundRemovalRecovered(
  const SegmentArray<PointT> & segmentArray,
  pcl::PointCloud<PointT> & removedPoints,
  const AlgorithmParameters & params = DEFAULT_ALGORITHM_PARAMETERS);

}  // namespace ground_removal

}  // namespace lidar

}  // namespace ugr

#include "ground_removal.tpp"

#endif  // GROUND_REMOVAL_HPP_
