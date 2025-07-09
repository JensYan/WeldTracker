#pragma once

#include <vector>  
#include "ImageMethod/Matrix.h"  

class PixelToLaserCoord {
public:
    // 构造函数
    PixelToLaserCoord();

    // 像素坐标转3D点（相机坐标系）
    std::vector<double> Get3DPoint(double r, double c);

    // 像素坐标转2D点（激光平面坐标系）
    std::vector<double> Get2DPoint(double r, double c);

    // 连续性滤波
    std::vector<double> ContinuityFilter(
        const std::vector<double>& currentPoint,
        const std::vector<std::vector<double>>& meaDatas
    );

    // 计算两点间欧氏距离
    double Cal_Dist(const std::vector<double>& pt1, const std::vector<double>& pt2);

private:
    // 相机内参
    double f = 0;
    double K = 0;
    double Sx = 0;
    double Sy = 0;
    double Cx = 0;
    double Cy = 0;

    // 激光平面方程系数
    double A = 0;
    double B = 0;
    double C = 0;
    double D = 0;

    // 预计算参数
    double P1 = 0, P2 = 0, P3 = 0, P4 = 0, P5 = 0, P6 = 0;

    // 生成平面变换矩阵
    Matrix<double> Pose_To_Mat3d();

    // 向量叉积
    std::vector<double> Cross(const std::vector<double>& a, const std::vector<double>& b);

    // 向量单位化
    std::vector<double> Norm(const std::vector<double>& a);
};