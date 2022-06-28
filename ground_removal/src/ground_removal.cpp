//===-- ugrdv_per...ion_lidar/src/ground_removal.cpp - Ground Removal Functions -*- C++ -*-===//
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

#include <cmath>
#include <cassert>
#include <vector>
#include <memory>
#include <algorithm>

#include "ground_removal.hpp"

namespace ugr
{

namespace lidar
{

namespace ground_removal
{

// vector2 constructors

Vector2::Vector2()
: Vector2(0, 0)
{
}

Vector2::Vector2(float x, float y)
: x(x), y(y)
{
}

// end vector2 constructors

// vector2 functions

// **see ground_removal.tpp for implementation of fromPCLPoint()**

float Vector2::sqrMagnitude() const
{
  return std::pow(x, 2) + std::pow(y, 2);
}

float Vector2::magnitude() const
{
  return std::sqrt(sqrMagnitude());
}

// end vector2 functions

// vector2 operators

Vector2 operator+(const Vector2 & lhs, const Vector2 & rhs)
{
  return Vector2(lhs.x + rhs.x, lhs.y + rhs.y);
}

Vector2 operator-(const Vector2 & lhs, const Vector2 & rhs)
{
  return lhs + (-1 * rhs);
}

Vector2 operator*(const Vector2 & lhs, const float & rhs)
{
  return Vector2(lhs.x * rhs, lhs.y * rhs);
}

Vector2 operator*(const float & lhs, const Vector2 & rhs)
{
  return operator*(rhs, lhs);
}

Vector2 operator/(const Vector2 & lhs, const float & rhs)
{
  return (1.0f / rhs) * lhs;
}

bool operator==(const Vector2 & lhs, const Vector2 & rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

// end vector2 operators

// line constructors

Line::Line(float gradient, float yIntercept, Vector2 beginPoint, Vector2 endPoint)
: gradient(gradient), yIntercept(yIntercept), beginPoint(beginPoint), endPoint(endPoint)
{
}

// end line constructors

// algorithm implementation

Line fitLine2D(const std::vector<Vector2> & points)
{
  assert(points.size() > 0);

  float sumX = 0.0f, sumY = 0.0f, sumXX = 0.0f, sumXY = 0.0f;

  for (const Vector2 & point : points) {
    sumX += point.x;
    sumY += point.y;
    sumXX += point.x * point.x;
    sumXY += point.x * point.y;
  }

  float count = points.size();
  float xMean = sumX / count, yMean = sumY / count;
  float slope = (sumXY - sumX * yMean) / (sumXX - sumX * xMean);
  float yIntercept = yMean - slope * xMean;

  // figure out which points are furthest back/front
  // by their x components
  Vector2 frontPoint = *std::min_element(
    points.begin(), points.end(),
    [](const Vector2 & a, const Vector2 & b)
    {return a.x < b.x;});

  Vector2 backPoint = *std::max_element(
    points.begin(), points.end(),
    [](const Vector2 & a, const Vector2 & b)
    {return a.x < b.x;});

  Vector2 startPoint(frontPoint.x, frontPoint.x * slope + yIntercept);
  Vector2 endPoint(backPoint.x, backPoint.x * slope + yIntercept);

  return Line(slope, yIntercept, startPoint, endPoint);
}

float distanceFromPointToLine(const Vector2 & point, const Line & line)
{
  Vector2 v1(point.x, point.y);
  Vector2 v2(point.x, point.x * line.gradient + line.yIntercept);
  return (v1 - v2).magnitude();
}

float fitRMSE(const Line & line, const std::vector<Vector2> & points)
{
  float sum = 0;
  float count = points.size();

  for (const Vector2 & point : points) {
    float x = point.x;
    float y = point.y;
    float yExpected = line.gradient * x + line.yIntercept;
    sum += std::pow(yExpected - y, 2);
  }

  return std::sqrt(sum / count);
}

// end algorithm implementation

}  // namespace ground_removal

}  // namespace lidar

}  // namespace ugr
