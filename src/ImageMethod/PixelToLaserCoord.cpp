#include "ImageMethod/PixelToLaserCoord.h"
#include <cmath>
#include <stdexcept>


// ���캯��
PixelToLaserCoord::PixelToLaserCoord()
    : f(0.00849243),
    K(-1471.27),
    Sx(5.30046e-006),
    Sy(5.3e-006),
    Cx(632.341),
    Cy(473.814),
    A(31.2674),
    B(0.709),
    C(-12.2524),
    D(1)
{
    // ��ʼ��Ԥ�������
    P1 = K * Cx * Cx * Sx * Sx;
    P2 = K * Cx * Sx * Sx;
    P3 = K * Cy * Cy * Sy * Sy;
    P4 = 2 * K * Cy * Sy * Sy;
    P5 = K * Sx * Sx;
    P6 = K * Sy * Sy;
}

// ��������ת3D�㣨�������ϵ��
std::vector<double> PixelToLaserCoord::Get3DPoint(double r, double c) {
    double kt = P1 - 2 * P2 * c + P3 - P4 * r + P5 * c * c + P6 * r * r + 1;
    double denominator = A * (Sx * c - Cx * Sx) + B * (Sy * r - Cy * Sy) + C * f * kt;

    // ���������
    if (std::fabs(denominator) < 1e-12) {
        throw std::runtime_error("Division by zero in 3D point calculation");
    }

    double x = -D * (Sx * c - Cx * Sx) / denominator;
    double y = -D * (Sy * r - Cy * Sy) / denominator;
    double z = -D * f * kt / denominator;

    return { x * 1000.0, y * 1000.0, z * 1000.0 };
}

// ��������ת2D�㣨����ƽ������ϵ��
std::vector<double> PixelToLaserCoord::Get2DPoint(double r, double c) {
    auto point3d = PixelToLaserCoord::Get3DPoint(r, c);

    // ת�����׵�λ
    point3d[0] /= 1000.0;
    point3d[1] /= 1000.0;
    point3d[2] /= 1000.0;

    // ��ȡ�任��������
    Matrix<double> tran = Pose_To_Mat3d();
    tran.InvertGaussJordan();

    // ���������������
    Matrix<double> pos(4, 1);
    pos(0, 0) = point3d[0];
    pos(1, 0) = point3d[1];
    pos(2, 0) = point3d[2];
    pos(3, 0) = 1.0;

    // Ӧ�ñ任
    Matrix<double> point2d_mat = tran * pos;

    // ���غ��׵�λ������
    return {
        point2d_mat(0, 0) * 1000.0,
        point2d_mat(1, 0) * 1000.0,
        point2d_mat(2, 0) * 1000.0
    };
}

// �������˲�
std::vector<double> PixelToLaserCoord::ContinuityFilter(
    const std::vector<double>& currentPoint,
    const std::vector<std::vector<double>>& meaDatas
) {
    // ��鵱ǰ��ά��
    if (currentPoint.size() != 3) {
        throw std::invalid_argument("currentPoint must be 3-dimensional");
    }
    if (meaDatas.empty()) {
        return currentPoint;
    }

    const auto& lastPoint = meaDatas.back();
    if (lastPoint.size() != 3) {
        throw std::invalid_argument("meaDatas contains invalid 3D points");
    }
    double alpha = 0.46;  // X/Y �˲�ϵ��
    double beta = 0.30;   // Z �˲�ϵ��

    return {
        (1 - alpha) * lastPoint[0] + alpha * currentPoint[0],
        (1 - alpha) * lastPoint[1] + alpha * currentPoint[1],
        (1 - beta) * lastPoint[2] + beta * currentPoint[2]
    };
}

// ���������ŷ�Ͼ���
double PixelToLaserCoord::Cal_Dist(const std::vector<double>& pt1, const std::vector<double>& pt2) {
    if (pt1.size() != 3 || pt2.size() != 3) {
        throw std::invalid_argument("input vector must be be 3-dimensional");
    }
    double dx = pt2[0] - pt1[0];
    double dy = pt2[1] - pt1[1];
    double dz = pt2[2] - pt1[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}



// ����ƽ��任����
Matrix<double> PixelToLaserCoord::Pose_To_Mat3d() {
    double t0 = -1.0 / (A * A + B * B + C * C);
    double t1 = -(C * 100.0 + 1.0) / (A * A + B * B + C * C);

    std::vector<double> p0 = { A * t0, B * t0, C * t0 };
    std::vector<double> p1 = { A * t1, B * t1, C * t1 + 100.0 };

    // ��������ϵ������
    std::vector<double> zb = {
        p1[0] - p0[0],
        p1[1] - p0[1],
        p1[2] - p0[2]
    };
    zb = Norm(zb);

    std::vector<double> yb = { A, B, C };
    yb = Norm(yb);

    std::vector<double> xb = Cross(yb, zb);

    // ����4x4��α任����
    Matrix<double> tran(4, 4);

    // ��һ��
    tran(0, 0) = xb[0];
    tran(0, 1) = yb[0];
    tran(0, 2) = zb[0];
    tran(0, 3) = p0[0];

    // �ڶ���
    tran(1, 0) = xb[1];
    tran(1, 1) = yb[1];
    tran(1, 2) = zb[1];
    tran(1, 3) = p0[1];

    // ������
    tran(2, 0) = xb[2];
    tran(2, 1) = yb[2];
    tran(2, 2) = zb[2];
    tran(2, 3) = p0[2];

    // ������
    tran(3, 0) = 0;
    tran(3, 1) = 0;
    tran(3, 2) = 0;
    tran(3, 3) = 1;

    return tran;
}

// �������
std::vector<double> PixelToLaserCoord::Cross(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != 3 || b.size() != 3) {
        throw std::invalid_argument("input vector must be be 3-dimensional");
    }
    return {
        a[1] * b[2] - a[2] * b[1],  // X����
        a[2] * b[0] - a[0] * b[2],  // Y����
        a[0] * b[1] - a[1] * b[0]   // Z����
    };
}

// ������λ��
std::vector<double> PixelToLaserCoord::Norm(const std::vector<double>& a) {
    if (a.size() != 3) {
        throw std::invalid_argument("input vector must be be 3-dimensional");
    }
    double len = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (len < 1e-12) {
        throw std::runtime_error("Attempt to normalize zero-length vector");
    }
    return { a[0] / len, a[1] / len, a[2] / len };
}
