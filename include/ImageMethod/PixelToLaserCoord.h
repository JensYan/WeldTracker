#pragma once

#include <vector>  
#include "ImageMethod/Matrix.h"  

class PixelToLaserCoord {
public:
    // ���캯��
    PixelToLaserCoord();

    // ��������ת3D�㣨�������ϵ��
    std::vector<double> Get3DPoint(double r, double c);

    // ��������ת2D�㣨����ƽ������ϵ��
    std::vector<double> Get2DPoint(double r, double c);

    // �������˲�
    std::vector<double> ContinuityFilter(
        const std::vector<double>& currentPoint,
        const std::vector<std::vector<double>>& meaDatas
    );

    // ���������ŷ�Ͼ���
    double Cal_Dist(const std::vector<double>& pt1, const std::vector<double>& pt2);

private:
    // ����ڲ�
    double f = 0;
    double K = 0;
    double Sx = 0;
    double Sy = 0;
    double Cx = 0;
    double Cy = 0;

    // ����ƽ�淽��ϵ��
    double A = 0;
    double B = 0;
    double C = 0;
    double D = 0;

    // Ԥ�������
    double P1 = 0, P2 = 0, P3 = 0, P4 = 0, P5 = 0, P6 = 0;

    // ����ƽ��任����
    Matrix<double> Pose_To_Mat3d();

    // �������
    std::vector<double> Cross(const std::vector<double>& a, const std::vector<double>& b);

    // ������λ��
    std::vector<double> Norm(const std::vector<double>& a);
};