#include "gtest/gtest.h"
#include "RobotMethod/LaserCoordToTcp.h"
#include <vector>
#include <stdexcept>

using namespace WeldTrackApp;

TEST(LaserCoordToTcpTest, Trans) {
	LaserCoordToTcp converter;
	// ���Է�����������״����
	EXPECT_THROW(
		converter.Cal_LaserMeaPtToBase(0.0, 0.0, {}),
		std::invalid_argument
	);
	EXPECT_THROW(
		converter.Cal_LaserMeaPtToBase(0.0, 0.0, { 1.0, 2.0, 3.0 }),
		std::invalid_argument
	);
	std::vector<double> point = converter.Cal_LaserMeaPtToBase(0.0, 0.0, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
	EXPECT_EQ(point.size(), 3);
	// ����ĵڶ��������޸�Ϊ�������������������ֵ��Ĭ��0.0������
	EXPECT_NEAR(point[0], 0.0, 1e-6);
	EXPECT_NEAR(point[1], 0.0, 1e-6);
	EXPECT_NEAR(point[2], 0.0, 1e-6);
}