#pragma once

#include <vector>
#include "ImageMethod/Matrix.h"
#include "WTrackDType.h"

namespace WeldTrackApp {
    class TrackAlgMethod {
    public:
        /// @brief ����ʾ�̹켣(����̬)���㺸�쳤�Ⱥͻ�������̬�ı仯
        /// @param mea_Pos �����㼯�� [[x, y, z, rx, ry, rz], ...] -> ��ʾ�̹켣
        /// @param IncAtt [out] ��̬�Ƕ������б� [[��rx, ��ry, ��rz], ...] (��)
        /// @param totalLen [out] �����ܳ��� (mm)
        /// @return ����ɹ����� true������ false
        bool Cal_WeldPara(const std::vector<std::vector<double>>& mea_Pos,
            std::vector<double>& IncAtt,
            double& totalLen);

        /// @brief ʾ�̸����м�����̬�Ƕȵ�����
        /// @param Start ��ʼ�Ƕ� (��)
        /// @param End �����Ƕ� (��)
        /// @return �Ƕ����� (��)
        double Cal_IncAtt(double Start, double End);

        /// @brief ��������֮���ŷ�Ͼ���
        /// @param firstPt ��һ�� [x, y, z]
        /// @param secondPt �ڶ��� [x, y, z]
        /// @return ������� (mm)
        double Cal_Length(const std::vector<double>& firstPt,
            const std::vector<double>& secondPt);

        /// @brief ���ٳ����м���岹��
        /// @param firstPt ��ʼ�� [x, y, z]
        /// @param secondPt ������ [x, y, z]
        /// @param TotalLen �����ܳ��� (mm)
        /// @param TotalIncAtt ����̬���� [��rx, ��ry, ��rz] (��)
        /// @return �岹�㼯�� [��x, ��y, ��z, ��rx, ��ry, ��rz] (��λ: mm/0.001, ��/0.0001)
        std::vector<std::vector<int>> Cal_InterPt(
            const std::vector<double>& firstPt,
            const std::vector<double>& secondPt,
            double TotalLen,
            const std::vector<double>& TotalIncAtt);

        /// @brief ���㺸������в������ܳ���
        /// @param trackDatas_Save ����ĸ������� [x, y, z, ...]
        /// @return �켣�ܳ��� (mm)
        double Cal_totalLength(const std::vector<std::vector<double>>& trackDatas_Save);

        /// @brief ���������켣�岹����
        /// @param In_trackDatas_Control [in/out] ����/����켣����
        /// @param totalLen �����ܳ��� (mm)
        /// @param totalIncAtt ����̬���� [��rx, ��ry, ��rz] (��)
        /// @return �岹���ݼ� [��x, ��y, ��z, ��rx, ��ry, ��rz]
        std::vector<std::vector<int>> Gen_BatchTrackIncPtData(
            std::vector<std::vector<double>>& In_trackDatas_Control,
            double totalLen,
            const std::vector<double>& totalIncAtt);

        /// @brief �ж�����֮���Ƿ���Ҫ�岹
        /// @param firstPt ��һ�� [x, y, z, ...]
        /// @param secondPt �ڶ��� [x, y, z, ...]
        /// @return ��Ҫ�岹���� true������ false
        bool Inter_Decision(const std::vector<double>& firstPt,
            const std::vector<double>& secondPt);

        /// @brief �������˲������������
        /// @param trackDatas_Save ԭʼ�������ݼ���
        /// @param MNoiseCov ��������Э����
        /// @param PNoiseCov ��������Э����
        /// @return �˲���ĵ� [x, y, z]
        std::vector<double> mea_Pos_Filter(
            const std::vector<std::vector<double>>& trackDatas_Save,
            double MNoiseCov,
            double PNoiseCov);

    private:
        /// @brief �������˲���ʵ��
        /// @param ilv_MeasureDatas ������������
        /// @param idv_MNoiseCov ��������Э����
        /// @param idv_PNoiseCov ��������Э����
        /// @return �˲������������
        std::vector<double> Cal_KalmanFilter(
            const std::vector<double>& ilv_MeasureDatas,
            double idv_MNoiseCov,
            double idv_PNoiseCov);
    };

} // namespace WeldTrackApp