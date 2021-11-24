#include <gtest/gtest.h>
#include "ground_removal.h"

using namespace ground_removal;

// vector tests

TEST(Vector, Construction)
{
	using v=Vector2;

	v v1(1,2);
	v v2(3,2);

	v v3(v1);
	EXPECT_EQ(v3,v1);
	v3 = v(v2);
	EXPECT_EQ(v3, v2);
}

TEST(Vector, Addition)
{
	using v=Vector2;
	
	EXPECT_EQ(v(1,3)+v(8,7),v(9,10));
	EXPECT_EQ(v(-4,-3)+v(1,2),v(-3,-1));
	EXPECT_EQ(v(0,0)+v(3,2),v(3,2));
	EXPECT_EQ(v(2.2,1.6)+v(2.2,1),v(4.4,2.6));

}

TEST(Vector, Subtraction)
{
	using v=Vector2;

	EXPECT_EQ(v(3,-2)-v(6,2),v(-3,-4));
	EXPECT_EQ(v(7,9)-v(3,2),v(4,7));
	EXPECT_EQ(v(8,0)-v(-1,-4),v(9,4));
}

TEST(Vector, Multiplication)
{
	using v=Vector2;

	EXPECT_EQ(v(1,0)*3,v(3,0));
	EXPECT_EQ(4*v(-2,0),v(-8,0));
	EXPECT_EQ(-3*v(-1,8),v(3,-24));
}

TEST(Vector, Division)
{
	using v=Vector2;
	EXPECT_EQ(v(1,0)/2,v(0.5,0));
	EXPECT_EQ(v(4,3)/3,v(4.0f/3.0f,1));
	EXPECT_EQ(v(-2,-4)/5,v(-2.0f/5.0f,-4.0f/5.0f));
}

TEST(Vector, Square_Magnitude)
{
	using v=Vector2;

	EXPECT_EQ(v(1,5).sqrMagnitude(), 26);
	EXPECT_EQ(v(4,-2).sqrMagnitude(), 20);
	EXPECT_EQ(v(-3,0).sqrMagnitude(), 9);
}

TEST(Vector, Magnitude)
{
	using v=Vector2;

	EXPECT_EQ(v(3,4).magnitude(), 5);
	EXPECT_EQ(v(-3,4).magnitude(), 5);
	EXPECT_EQ(v(3, 0).magnitude(), 3);
	EXPECT_EQ(v(0,0).magnitude(), 0);
}

// end of vector tests

// algorithm function tests

TEST(Ground_Removal, fitRMSE)
{
	const float range = 0.001f;

	using v=Vector2;
	using l=Line;
	using std::vector;

	vector<v> points = {v(2,3), v(3,7), v(4,8), v(5,9), v(6,10)};
	l line = l(4, 2, v(), v());

	EXPECT_NEAR(fitRMSE(line, points), 11.16244, range);

	points = {v(-3,8), v(0,7), v(2,2), v(4,6), v(6,3), v(7,4), v(12,-9)};
	line = l(2,12,v(),v());

	EXPECT_NEAR(fitRMSE(line, points), 21.94474, range);

	points = {v(-10,1), v(-2,2), v(0,3), v(4,4), v(7,4), v(8,5)};
	line = l(0,3,v(),v());

	EXPECT_NEAR(fitRMSE(line, points), 1.35401, range);

}

TEST(Ground_Removal, distanceFromPointToLine)
{
	using v=Vector2;
	using l=Line;

	const v z;

	v point(3,9);
	l line(4,-2,z,z);

	EXPECT_EQ(distanceFromPointToLine(point, line), 1);

	point = v(-7,9);
	line = l(3,8,z,z);

	EXPECT_EQ(distanceFromPointToLine(point, line), 22);
	 
	point = v(-12,8);
	line = l(2,8,z,z);

	EXPECT_EQ(distanceFromPointToLine(point, line), 24);

	point = v(0,4);
	line = l(0,0,z,z);

	EXPECT_EQ(distanceFromPointToLine(point, line), 4);

	point = v(-6,2);
	line = l(3,9,z,z);

	EXPECT_EQ(distanceFromPointToLine(point, line), 11);
}

TEST(Ground_Removal, fitLine2D)
{
	const float range = 0.001f;

	using v=Vector2;
	using l=Line;
	using std::vector;

	vector<v> points = {v(-5,-6), v(-4,-4), v(-3,-5), v(-2,-3), v(0,0), v(1,3), v(4,2)};
	l fit = fitLine2D(points);

	EXPECT_NEAR(fit.gradient, 1.03125, range);
	EXPECT_NEAR(fit.yIntercept, -0.53125, range);

	points = {v(-7,3.6),v(-5,2.9),v(-12,2.6),v(-14,2.6),v(-19,1.5),v(-23,1.05),v(-50,-0.89)};
	fit = fitLine2D(points);

	EXPECT_NEAR(fit.gradient, 0.09546, range);
	EXPECT_NEAR(fit.yIntercept, 3.68143, range);

	points = {v(-10,36),v(-8,42),v(-6,57),v(-4,86),v(-2,97),v(0,107)};
	fit = fitLine2D(points);

	EXPECT_NEAR(fit.gradient, 7.84286, range);
	EXPECT_NEAR(fit.yIntercept, 110.04762, range);

}

TEST(Ground_Removal, prototypePoint)
{

	using point=pcl::PointXYZ;
	using v=Vector2;
	using p=pcl::PointXYZ;

	PointArray<p> arr = {new p(3,4,2), new p(0,0,0), new p(4,52,5)};

	EXPECT_EQ(prototypePoint(arr), Vector2::fromPclPoint(p(0,0,0)));	

	for(auto x : arr)
		delete x;

	arr = {new p(9,0,5), new p(-3,-4,0.005),new p(-9,-5,12)};

	EXPECT_EQ(prototypePoint(arr), Vector2::fromPclPoint(p(-3,-4,0.005)));

	for(auto x : arr)
		delete x;


}

// end of algorithm function tests
