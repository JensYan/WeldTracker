#pragma once

#include <vector>
#include <memory>
#include "ImageMethod/Matrix.h"

// ǰ������
class PixelToLaserCoord;

namespace WeldTrackApp {
    class LaserCoordToTcp {
    public:
        // ���캯��
        LaserCoordToTcp();
        // ǰ������->������һ��ָ��->��Ҫ����ָ������->ʵ����������
        ~LaserCoordToTcp();

        std::vector<double> Cal_LaserMeaPtToBase(double r, double c, const std::vector<double>& FLPPoint);

    private:
        // ����ƽ������ϵ�궨���-�������۱궨
        std::vector<double> LaserCoord;
        // ��������ϵ��������
        Matrix<double> LaserCoordToFLP;
        // ������������ϵ����������ϵ
        std::unique_ptr<PixelToLaserCoord> thePixelToLaserCoord;

        // ���峣��Pi
        static constexpr double PI = 3.14159265358979323846;

        Matrix<double> Cal_TCPTranMat(const std::vector<double>& In_TCPCoord);

        Matrix<double> Cal_LaserTranMat(const std::vector<double>& In_LaserCoord);
    };

} // namespace WeldTrackApp