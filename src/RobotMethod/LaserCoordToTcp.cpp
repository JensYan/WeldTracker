#include "RobotMethod/LaserCoordToTcp.h"
#include "ImageMethod/PixelToLaserCoord.h"  // 实际实现需要包含
#include "WTrackDType.h"  // 包含宏定义和枚举
#include <cmath>
#include <stdexcept>

namespace WeldTrackApp {
    LaserCoordToTcp::LaserCoordToTcp()
        : thePixelToLaserCoord(std::make_unique<PixelToLaserCoord>()),
          LaserCoord({ 0.0899, -0.7703, 0.9834, 0.0267, 0.0701, 0.5472, 46.9476, -1.9444, 351.2831 })
    {
        // 计算激光平面坐标系到法兰盘的变换矩阵
        LaserCoordToFLP = Cal_LaserTranMat(LaserCoord);
    }

    LaserCoordToTcp::~LaserCoordToTcp() = default;

    std::vector<double> LaserCoordToTcp::Cal_LaserMeaPtToBase(double r, double c, const std::vector<double>& FLPPoint) {
        std::vector<double> LaserPoint = thePixelToLaserCoord->Get2DPoint(r, c);
        if(LaserPoint.size() != 3) {
            throw std::invalid_argument("Size of LaserPoint must be 3-dimensional");
        }

        // 创建齐次坐标点 [X, 0, Z, 1]
        Matrix<double> MeaPt(4, 1);
        MeaPt(0, 0) = LaserPoint[0];
        MeaPt(1, 0) = 0;  // Y坐标为0（激光平面）
        MeaPt(2, 0) = LaserPoint[2];
        MeaPt(3, 0) = 1;  // 齐次坐标

        // 转换到法兰盘坐标系
        Matrix<double> MeaToTCP = LaserCoordToFLP * MeaPt;

        // 转换到基坐标系
        Matrix<double> FLPCoordToBase = Cal_TCPTranMat(FLPPoint);
        Matrix<double> Rt_Mat = FLPCoordToBase * MeaToTCP;

        // 提取结果并应用补偿
        std::vector<double> Rt_Array(3);
        Rt_Array[0] = Rt_Mat(0, 0) + MacroDefine::Cab_Corr_X;
        Rt_Array[1] = Rt_Mat(1, 0) + MacroDefine::Cab_Corr_Y;
        Rt_Array[2] = Rt_Mat(2, 0) + MacroDefine::Cab_Corr_Z;

        return Rt_Array;
    }

    Matrix<double> LaserCoordToTcp::Cal_TCPTranMat(const std::vector<double>& In_TCPCoord) {
        // 确保输入包含6个元素 [x, y, z, rz, ry, rx]
        if (In_TCPCoord.size() != 6) {
            throw std::invalid_argument("In_TCPCoord must have at least 6 elements");
        }

        // 角度转换 (度->弧度)
        double Rx = In_TCPCoord[5] * PI / 180.0;  // rx
        double Ry = In_TCPCoord[4] * PI / 180.0;  // ry
        double Rz = In_TCPCoord[3] * PI / 180.0;  // rz

        double ca = std::cos(Rx);  // cos(rx)
        double sa = std::sin(Rx);  // sin(rx)
        double cb = std::cos(Ry);  // cos(ry)
        double sb = std::sin(Ry);  // sin(ry)
        double cc = std::cos(Rz);  // cos(rz)
        double sc = std::sin(Rz);  // sin(rz)

        // 创建4x4齐次变换矩阵
        Matrix<double> Tran_matrix(4, 4);

        // 旋转部分 (R = Rz * Ry * Rx)
        Tran_matrix(0, 0) = ca * cb;
        Tran_matrix(0, 1) = ca * sb * sc - sa * cc;
        Tran_matrix(0, 2) = ca * sb * cc + sa * sc;

        Tran_matrix(1, 0) = sa * cb;
        Tran_matrix(1, 1) = sa * sb * sc + ca * cc;
        Tran_matrix(1, 2) = sa * sb * cc - ca * sc;

        Tran_matrix(2, 0) = -sb;
        Tran_matrix(2, 1) = cb * sc;
        Tran_matrix(2, 2) = cb * cc;

        // 平移部分
        Tran_matrix(0, 3) = In_TCPCoord[0];  // X
        Tran_matrix(1, 3) = In_TCPCoord[1];  // Y
        Tran_matrix(2, 3) = In_TCPCoord[2];  // Z

        // 齐次坐标最后一行
        Tran_matrix(3, 0) = 0;
        Tran_matrix(3, 1) = 0;
        Tran_matrix(3, 2) = 0;
        Tran_matrix(3, 3) = 1;

        return Tran_matrix;
    }

    Matrix<double> LaserCoordToTcp::Cal_LaserTranMat(const std::vector<double>& In_LaserCoord) {
        // 确保输入包含9个元素
        if (In_LaserCoord.size() != 9) {
            throw std::invalid_argument("In_LaserCoord must have 9 elements");
        }

        // 创建4x4齐次变换矩阵
        Matrix<double> Tran_matrix(4, 4);

        // 旋转部分 (3x3)
        Tran_matrix(0, 0) = In_LaserCoord[0];  // R11
        Tran_matrix(0, 1) = 0;                 // R12 (设置为0)
        Tran_matrix(0, 2) = In_LaserCoord[1];  // R13

        Tran_matrix(1, 0) = In_LaserCoord[2];  // R21
        Tran_matrix(1, 1) = 0;                 // R22 (设置为0)
        Tran_matrix(1, 2) = In_LaserCoord[3];  // R23

        Tran_matrix(2, 0) = In_LaserCoord[4];  // R31
        Tran_matrix(2, 1) = 0;                 // R32 (设置为0)
        Tran_matrix(2, 2) = In_LaserCoord[5];  // R33

        // 平移部分
        Tran_matrix(0, 3) = In_LaserCoord[6];  // X
        Tran_matrix(1, 3) = In_LaserCoord[7];  // Y
        Tran_matrix(2, 3) = In_LaserCoord[8];  // Z

        // 齐次坐标最后一行
        Tran_matrix(3, 0) = 0;
        Tran_matrix(3, 1) = 0;
        Tran_matrix(3, 2) = 0;
        Tran_matrix(3, 3) = 1;

        return Tran_matrix;
    }

} // namespace WeldTrackApp