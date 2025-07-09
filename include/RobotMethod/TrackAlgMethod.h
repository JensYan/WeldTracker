#pragma once

#include <vector>
#include "ImageMethod/Matrix.h"
#include "WTrackDType.h"

namespace WeldTrackApp {
    class TrackAlgMethod {
    public:
        /// @brief 根据示教轨迹(含姿态)计算焊缝长度和机器人姿态的变化
        /// @param mea_Pos 测量点集合 [[x, y, z, rx, ry, rz], ...] -> 即示教轨迹
        /// @param IncAtt [out] 姿态角度增量列表 [[Δrx, Δry, Δrz], ...] (度)
        /// @param totalLen [out] 焊缝总长度 (mm)
        /// @return 计算成功返回 true，否则 false
        bool Cal_WeldPara(const std::vector<std::vector<double>>& mea_Pos,
            std::vector<double>& IncAtt,
            double& totalLen);

        /// @brief 示教跟踪中计算姿态角度的增量
        /// @param Start 起始角度 (度)
        /// @param End 结束角度 (度)
        /// @return 角度增量 (度)
        double Cal_IncAtt(double Start, double End);

        /// @brief 计算两点之间的欧氏距离
        /// @param firstPt 第一点 [x, y, z]
        /// @param secondPt 第二点 [x, y, z]
        /// @return 两点距离 (mm)
        double Cal_Length(const std::vector<double>& firstPt,
            const std::vector<double>& secondPt);

        /// @brief 跟踪程序中计算插补点
        /// @param firstPt 起始点 [x, y, z]
        /// @param secondPt 结束点 [x, y, z]
        /// @param TotalLen 焊缝总长度 (mm)
        /// @param TotalIncAtt 总姿态增量 [Δrx, Δry, Δrz] (度)
        /// @return 插补点集合 [Δx, Δy, Δz, Δrx, Δry, Δrz] (单位: mm/0.001, 度/0.0001)
        std::vector<std::vector<int>> Cal_InterPt(
            const std::vector<double>& firstPt,
            const std::vector<double>& secondPt,
            double TotalLen,
            const std::vector<double>& TotalIncAtt);

        /// @brief 计算焊缝过程中测量的总长度
        /// @param trackDatas_Save 保存的跟踪数据 [x, y, z, ...]
        /// @return 轨迹总长度 (mm)
        double Cal_totalLength(const std::vector<std::vector<double>>& trackDatas_Save);

        /// @brief 生成批量轨迹插补数据
        /// @param In_trackDatas_Control [in/out] 输入/输出轨迹数据
        /// @param totalLen 焊缝总长度 (mm)
        /// @param totalIncAtt 总姿态增量 [Δrx, Δry, Δrz] (度)
        /// @return 插补数据集 [Δx, Δy, Δz, Δrx, Δry, Δrz]
        std::vector<std::vector<int>> Gen_BatchTrackIncPtData(
            std::vector<std::vector<double>>& In_trackDatas_Control,
            double totalLen,
            const std::vector<double>& totalIncAtt);

        /// @brief 判断两点之间是否需要插补
        /// @param firstPt 第一点 [x, y, z, ...]
        /// @param secondPt 第二点 [x, y, z, ...]
        /// @return 需要插补返回 true，否则 false
        bool Inter_Decision(const std::vector<double>& firstPt,
            const std::vector<double>& secondPt);

        /// @brief 卡尔曼滤波处理测量数据
        /// @param trackDatas_Save 原始测量数据集合
        /// @param MNoiseCov 测量噪声协方差
        /// @param PNoiseCov 过程噪声协方差
        /// @return 滤波后的点 [x, y, z]
        std::vector<double> mea_Pos_Filter(
            const std::vector<std::vector<double>>& trackDatas_Save,
            double MNoiseCov,
            double PNoiseCov);

    private:
        /// @brief 卡尔曼滤波器实现
        /// @param ilv_MeasureDatas 测量数据数组
        /// @param idv_MNoiseCov 测量噪声协方差
        /// @param idv_PNoiseCov 过程噪声协方差
        /// @return 滤波后的数据数组
        std::vector<double> Cal_KalmanFilter(
            const std::vector<double>& ilv_MeasureDatas,
            double idv_MNoiseCov,
            double idv_PNoiseCov);
    };

} // namespace WeldTrackApp