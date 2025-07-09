#include "ImageMethod/PixelToLaserCoord.h"
#include "gtest/gtest.h"
#include <vector>

TEST(PTLCTest, Get3DPoint) {
	PixelToLaserCoord ptlc;
	// 已知点的转换
	std::vector<double> point = ptlc.Get3DPoint(0, 0);
	ASSERT_EQ(point.size(), 3);
	EXPECT_NEAR(point[0], -16.1181, 1e-6);
	EXPECT_NEAR(point[1], -12.0762, 1e-6);
	EXPECT_NEAR(point[2], 39.7855, 1e-6);
}

TEST(PTLCTest, Get2DPoint) {
	PixelToLaserCoord ptlc;
	std::vector<double> point = ptlc.Get2DPoint(0, 0);
	ASSERT_EQ(point.size(), 3);
	EXPECT_NEAR(point[0], -13.57, 1e-2); // 需要在C#得到这个值
	EXPECT_NEAR(point[1], 15.56, 1e-2);  // 现在是瞎填的
	EXPECT_NEAR(point[2], 52.32, 1e-2);
}

TEST(PTLCTest, ContinuityFilter) {
	PixelToLaserCoord ptlc;
	std::vector<double> point1 = ptlc.ContinuityFilter({ 0, 0, 0 }, {});
	ASSERT_EQ(point1.size(), 3);
	EXPECT_NEAR(point1[0], 0, 1e-10);
	EXPECT_NEAR(point1[1], 0, 1e-10);
	EXPECT_NEAR(point1[2], 0, 1e-10);

	EXPECT_THROW(
		ptlc.ContinuityFilter({ 0, 0 ,0 }, { {1, 2, 3}, {1, 2} }),
		std::invalid_argument
	);
	EXPECT_THROW(
		ptlc.ContinuityFilter({ 0, 0 ,0 }, { {1, 2}, {} }),
		std::invalid_argument
	);
	EXPECT_THROW(
		ptlc.ContinuityFilter({ 0, 0 }, { {1, 2, 3} }),
		std::invalid_argument
	);

}