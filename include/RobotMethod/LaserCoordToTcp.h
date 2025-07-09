#pragma once

#include <vector>
#include <memory>
#include "ImageMethod/Matrix.h"

// 前向声明
class PixelToLaserCoord;

namespace WeldTrackApp {
    class LaserCoordToTcp {
    public:
        // 构造函数
        LaserCoordToTcp();
        // 前向声明->声明了一个指针->需要管理指针周期->实现析构函数
        ~LaserCoordToTcp();

        std::vector<double> Cal_LaserMeaPtToBase(double r, double c, const std::vector<double>& FLPPoint);

    private:
        // 激光平面坐标系标定结果-来自手眼标定
        std::vector<double> LaserCoord;
        // 激光坐标系到法兰盘
        Matrix<double> LaserCoordToFLP;
        // 计算像素坐标系到激光坐标系
        std::unique_ptr<PixelToLaserCoord> thePixelToLaserCoord;

        // 定义常量Pi
        static constexpr double PI = 3.14159265358979323846;

        Matrix<double> Cal_TCPTranMat(const std::vector<double>& In_TCPCoord);

        Matrix<double> Cal_LaserTranMat(const std::vector<double>& In_LaserCoord);
    };

} // namespace WeldTrackApp