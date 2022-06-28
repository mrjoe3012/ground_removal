//===-- ugrdv_per...ion_lidar/include/ground_removal.tpp - Ground Removal Functions -*- C++ -*-===//
//
// Part of the UGRDV Project, under the UGRACING DRIVERLESS PRIVATE LICENSE.
// See the attached LICENSE.txt file in this projects upper most directory for
// license information.
//
// Copyright Joseph Agrane © 2021-2022
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains the  ground removal functions which come after filtering.
///
//===----------------------------------------------------------------------===//

#ifndef GROUND_REMOVAL_TPP_
#define GROUND_REMOVAL_TPP_

#include <cmath>
#include <cassert>
#include <vector>
#include <memory>
#include <algorithm>

namespace ugr
{

  namespace lidar
  {

    namespace ground_removal
    {

// vector functions

      template < typename PointT >
      Vector2 Vector2::fromPclPoint(const PointT & p)
      {
        return Vector2(std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2)), p.z);
      }

// end vector functions

// algorithm implementation

      template < typename PointT >
      std::unique_ptr < SegmentArray < PointT >> assignPointsToBinsAndSegments(
        const pcl::PointCloud < PointT > &cloud, int numberOfSegments, int numberOfBins)
      {
        assert(numberOfSegments > 0);
        assert(numberOfBins > 0);

        if (cloud.size() == 0) {
          return std::unique_ptr < SegmentArray < PointT >> (new SegmentArray < PointT > ());
        }

        const float pi = 4 * std::atan(1);

        // construct the result so that it already contains the
        // correct number of arrays for segments and bins
        std::unique_ptr < SegmentArray < PointT >> pSegmentArray(
          new SegmentArray < PointT > (numberOfSegments, BinArray < PointT > (numberOfBins)));

        // figure out how far away the farthest point is
        // so that we can determine how big each bin should
        // be.
        // Note that using the distance squared is acceptable here
        const PointT & farthestPoint = *std::max_element(
          cloud.begin(), cloud.end(), [] (const PointT & p1, const PointT & p2)
          {return std::pow(p1.x, 2) + std::pow(p1.y, 2) < std::pow(p2.x, 2) + std::pow(p2.y, 2);});

        float farthestPointDistance =
          std::sqrt(std::pow(farthestPoint.x, 2) + std::pow(farthestPoint.y, 2));

        // distance between edge of each bin
        float binRange = farthestPointDistance / numberOfBins;
        // angle between each segment edge
        float segmentAngle = (2.0f * pi) / numberOfSegments;

        for (const PointT & point : cloud) {
          float pointDistance = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));
          float pointAngleWithXAxis = std::atan2(point.y, point.x);
          if (pointAngleWithXAxis < 0) {
            pointAngleWithXAxis = 2.0f * pi + pointAngleWithXAxis;
          }

          unsigned int segmentIndex = std::min(
            static_cast < unsigned int > (pointAngleWithXAxis / segmentAngle),
            static_cast < unsigned int > (numberOfSegments - 1));

          unsigned int binIndex = std::min(
            static_cast < unsigned int > (pointDistance / binRange),
            static_cast < unsigned int > (numberOfBins - 1));

          // assign the bin according to its index
          (*pSegmentArray)[segmentIndex][binIndex].push_back(&point);
        }

        return pSegmentArray;
      }

      template < typename PointT >
      Vector2 prototypePoint(const PointArray < PointT > & pointArray)
      {
        assert(pointArray.size() > 0);
        const PointT * pPoint = *std::min_element(
          pointArray.begin(), pointArray.end(),
          [] (const PointT * lhs, const PointT * rhs)
          {return lhs->z < rhs->z;});
        // convert 3d point into 2d point
        Vector2 point2d = Vector2::fromPclPoint(*pPoint);
        return point2d;
      }

      template < typename PointT >
      std::vector < Line > groundPlaneLinesForSegment(
        const BinArray < PointT > &binArray,
        const AlgorithmParameters & params)
      {
        float tM = params.tM, tMSmall = params.tMSmall;
        float tB = params.tB;
        float tRMSE = params.tRMSE;
        float tDPrev = params.tDPrev;

        // points which are incrementally populated and
        // are ultimately used to fit ground plane lines
        std::vector < Vector2 > linePoints;
        // the ground plane lines
        std::vector < Line > lines;

        unsigned int numberOfBins = binArray.size();

        for (unsigned int i = 0, c = 0; i < numberOfBins; i++) {
          const std::vector < const PointT * > & pointArray = binArray[i];

          if (pointArray.size() > 0) {
            Vector2 prototype = prototypePoint(pointArray);
            if (linePoints.size() >= 2) {
              linePoints.push_back(prototype);
              Line line = fitLine2D(linePoints);
              if (!(std::abs(line.gradient) <= tM &&
                (line.gradient > tMSmall ||
                std::abs(line.yIntercept) <= tB) &&
                fitRMSE(line, linePoints) <= tRMSE))
              {
                linePoints.pop_back();
                line = fitLine2D(linePoints);
                lines.push_back(line);
                linePoints.clear();
                c++;
                i--;
              }
            } else {
              if (linePoints.size() != 0 ||
                c == 0 ||
                distanceFromPointToLine(prototype, lines[c - 1]) <= tDPrev)
              {
                linePoints.push_back(prototype);
              }
            }
          }
        }

        return lines;
      }

      template < typename PointT >
      PCPtr < PointT > groundRemoval(
        const SegmentArray < PointT > &segmentArray,
        const AlgorithmParameters & params)
      {
        // store the removed points, but don't actually do anything with them
        PCPtr < PointT > pRemovedPoints = PCPtr < PointT > (new pcl::PointCloud < PointT > ());
        PCPtr < PointT > result = groundRemovalRecovered(segmentArray, *pRemovedPoints, params);
        return result;
      }

      template < typename PointT >
      PCPtr < PointT > groundRemovalRecovered(
        const SegmentArray < PointT > &segmentArray,
        pcl::PointCloud < PointT > &removedPoints,
        const AlgorithmParameters & params)
      {
        float tDGround = params.tDGround;

        PCPtr < PointT > result(new pcl::PointCloud < PointT >);

        unsigned int segmentIndex = 0;

        for (const BinArray < PointT > & binArray : segmentArray) {
          std::vector < Line > groundPlaneLines = groundPlaneLinesForSegment(binArray, params);
          for (const PointArray < PointT > & pointArray : binArray) {
            for (const PointT * point : pointArray) {
              Vector2 point2d = Vector2::fromPclPoint(*point);
              bool groundPoint = false;

              // find the closest line to the ground point
              const Line * closestLine = nullptr;
              float distanceToClosestLine = std::numeric_limits < float > ::max();
              for (const Line & line : groundPlaneLines) {
                float distanceToStart = (line.beginPoint - point2d).sqrMagnitude();
                float distanceToEnd = (line.endPoint - point2d).sqrMagnitude();
                if (distanceToEnd < distanceToClosestLine ||
                  distanceToStart < distanceToClosestLine)
                {
                  closestLine = &line;
                  distanceToClosestLine = std::min(distanceToStart, distanceToEnd);
                }
              }

              // check the point's minimum distance to the closest line against
              // the ground point distance threshold to determine whether or not we should remove it
              if (closestLine != nullptr) {
                float distanceToLine = distanceFromPointToLine(point2d, *closestLine);
                if (distanceToLine <= tDGround) {
                  groundPoint = true;
                }
              }

              // add the point to the new cloud only if it isn't a ground point
              if (!groundPoint) {
                result->push_back(*point);
              } else {
                removedPoints.push_back(*point);
              }
            }
          }
          segmentIndex++;
        }

        return result;
      }

// end algorithm implementation

    } // namespace ground_removal

  } // namespace lidar

}  // namespace ugr

#endif  // GROUND_REMOVAL_TPP_
