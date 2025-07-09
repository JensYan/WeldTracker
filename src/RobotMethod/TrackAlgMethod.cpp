#include "RobotMethod/TrackAlgMethod.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <stdexcept>

namespace WeldTrackApp {

    bool TrackAlgMethod::Cal_WeldPara(
        const std::vector<std::vector<double>>& mea_Pos,
        std::vector<double>& IncAtt,
        double& totalLen)
    {
        // ��ʼ���������
        IncAtt.clear();
        IncAtt.resize(3, 0.0);
        totalLen = 0.0;

        if (mea_Pos.size() < 2) {
            return false;
        }

        // ��ȡ���ͽ��������̬
        const auto& startPoint = mea_Pos[0];
        const auto& endPoint = mea_Pos.back();

        double startAtt[3] = { startPoint[3], startPoint[4], startPoint[5] };
        double endAtt[3] = { endPoint[3], endPoint[4], endPoint[5] };

        // ������̬����
        IncAtt[0] = Cal_IncAtt(startAtt[0], endAtt[0]);
        IncAtt[1] = Cal_IncAtt(startAtt[1], endAtt[1]);
        IncAtt[2] = Cal_IncAtt(startAtt[2], endAtt[2]);

        // ���㺸�ӹ켣����
        for (size_t i = 0; i < mea_Pos.size() - 1; ++i) {
            double len = Cal_Length(mea_Pos[i], mea_Pos[i + 1]);
            totalLen += len;
        }

        return true;
    }

    double TrackAlgMethod::Cal_IncAtt(double Start, double End) {
        // 1. ����ԭʼ�ǶȲ�
        double diff = End - Start;

        // 2. ��һ����[-180��, 180��]����
        diff = std::fmod(diff, 360.0); // ��ȡģ��(-360, 360)

        // 3. ������[0, 360)��Χ
        if (diff < 0.0) {
            diff += 360.0;
        }
        // 4. ӳ�䵽[-180��, 180��]
        if (diff > 180.0) {
            diff -= 360.0;
        }
        return diff;
    }

    double TrackAlgMethod::Cal_Length(
        const std::vector<double>& firstPt,
        const std::vector<double>& secondPt)
    {
        // ȷ����������Ч
        if (firstPt.size() < 3 || secondPt.size() < 3) {
            throw std::invalid_argument("input point must be 3 - dimensional");
        }

        double dx = secondPt[0] - firstPt[0];
        double dy = secondPt[1] - firstPt[1];
        double dz = secondPt[2] - firstPt[2];

        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    std::vector<std::vector<int>> TrackAlgMethod::Cal_InterPt(
        const std::vector<double>& firstPt,
        const std::vector<double>& secondPt,
        double TotalLen,
        const std::vector<double>& TotalIncAtt)
    {
        std::vector<std::vector<int>> Inter_Pts;

        // �����������
        double Len = Cal_Length(firstPt, secondPt);
        
        if (Len > TotalLen) {
            throw std::invalid_argument("TotalLen less than distance between points");
        }

        // ����岹������
        int InterNum = static_cast<int>(Len / (MacroDefine::WeldSpeed * MacroDefine::InterCycle));

        if (InterNum == 0) {
            return Inter_Pts;
        }

        // ����ÿ���岹��֮�����̬����
        std::vector<int> CurIncAtt(TotalIncAtt.size(), 0);

        if (TotalLen > 0) {
            for (size_t i = 0; i < TotalIncAtt.size(); ++i) {
                double inc = TotalIncAtt[i] * Len / (TotalLen * InterNum);
                CurIncAtt[i] = static_cast<int>(inc * 10000);  // ת��Ϊ 0.0001 �ȵ�λ
            }
        }
        else {
            throw std::invalid_argument("TotalLen must be bigger than 0");
        }

        // ����ÿ���岹��֮���λ������
        std::vector<int> CurIncPos(3, 0);
        for (int i = 0; i < 3; ++i) {
            double inc = (secondPt[i] - firstPt[i]) / InterNum;
            CurIncPos[i] = static_cast<int>(inc * 1000);  // ת��Ϊ 0.001 mm ��λ
        }

        // ���ɲ岹��
        for (int i = 0; i < InterNum; ++i) {
            std::vector<int> Inter_Pt(6, 0);
            Inter_Pt[0] = CurIncPos[0];
            Inter_Pt[1] = CurIncPos[1];
            Inter_Pt[2] = 0;  // Z��������Ϊ0����ȫ���ǣ�
            Inter_Pt[3] = CurIncAtt[0];
            Inter_Pt[4] = CurIncAtt[1];
            Inter_Pt[5] = CurIncAtt[2];

            Inter_Pts.push_back(Inter_Pt);
        }

        return Inter_Pts;
    }

    double TrackAlgMethod::Cal_totalLength(
        const std::vector<std::vector<double>>& trackDatas_Save)
    {
        double mea_path_Len = 0.0;

        if (trackDatas_Save.size() < 2) {
            return mea_path_Len;
        }

        for (size_t i = 0; i < trackDatas_Save.size() - 1; ++i) {
            // ��ȡ���������λ��
            const auto& pt1 = trackDatas_Save[i];
            const auto& pt2 = trackDatas_Save[i + 1];

            // ȷ�����ݸ�ʽ��ȷ
            if (pt1.size() >= 3 && pt2.size() >= 3) {
                // ��Ӧ�������б������ݵ�(x, y, z) ά�� // ��ȷ��
                std::vector<double> pos1 = { pt1[5], pt1[6], pt1[7] };
                std::vector<double> pos2 = { pt2[5], pt2[6], pt2[7] };

                double len = Cal_Length(pos1, pos2);
                mea_path_Len += len;
            }
            else {
                throw std::invalid_argument("trackDatas_Save contains invalid datas");
            }
        }

        return mea_path_Len;
    }

    std::vector<std::vector<int>> TrackAlgMethod::Gen_BatchTrackIncPtData(
        std::vector<std::vector<double>>& In_trackDatas_Control,
        double totalLen,
        const std::vector<double>& totalIncAtt)
    {
        std::vector<std::vector<int>> all_IncDatas;

        if (In_trackDatas_Control.size() < 2) {
            return all_IncDatas;
        }

        // �������еĲ岹���ݼ���ȥ��������̵ĵ㣩
        std::vector<std::vector<double>> trackDatas_Control_Feasible;
        trackDatas_Control_Feasible.push_back(In_trackDatas_Control[0]);

        for (size_t i = 1; i < In_trackDatas_Control.size(); ++i) {
            const auto& firstPt = trackDatas_Control_Feasible.back();
            const auto& secondPt = In_trackDatas_Control[i];

            if (Inter_Decision(firstPt, secondPt)) {
                trackDatas_Control_Feasible.push_back(secondPt);
            }
        }

        // ����岹��������
        if (trackDatas_Control_Feasible.size() > 1) {
            for (size_t i = 0; i < trackDatas_Control_Feasible.size() - 1; ++i) {
                auto interPts = Cal_InterPt(
                    trackDatas_Control_Feasible[i],
                    trackDatas_Control_Feasible[i + 1],
                    totalLen,
                    totalIncAtt);

                all_IncDatas.insert(all_IncDatas.end(), interPts.begin(), interPts.end());
            }
            // �Ƴ��Ѵ��������
            if (!In_trackDatas_Control.empty()) {
                In_trackDatas_Control.erase(In_trackDatas_Control.begin(),
                    In_trackDatas_Control.begin() + In_trackDatas_Control.size() - 1);
            }
        }
        return all_IncDatas;
    }

    bool TrackAlgMethod::Inter_Decision(
        const std::vector<double>& firstPt,
        const std::vector<double>& secondPt)
    {
        double Len = Cal_Length(firstPt, secondPt);
        int InterNum = static_cast<int>(Len / (MacroDefine::WeldSpeed * MacroDefine::InterCycle));
        return (InterNum > 0);
    }

    std::vector<double> TrackAlgMethod::mea_Pos_Filter(
        const std::vector<std::vector<double>>& trackDatas_Save,
        double MNoiseCov,
        double PNoiseCov)
    {
        std::vector<double> x_trackDatas_Save;
        std::vector<double> y_trackDatas_Save;
        std::vector<double> z_trackDatas_Save;

        // ��ȡ XYZ ��������  // ��ȷ��
        for (const auto& data : trackDatas_Save) {
            if (data.size() >= 3) {
                x_trackDatas_Save.push_back(data[0]);
                y_trackDatas_Save.push_back(data[1]);
                z_trackDatas_Save.push_back(data[2]);
            }
        }

        size_t cur_Count = x_trackDatas_Save.size();
        if (cur_Count == 0) {
            return { 0.0, 0.0, 0.0 };
        }

        // �������ݲ��㲿��
        for (size_t i = 0; i < MacroDefine::filterDelay - cur_Count; ++i) {
            x_trackDatas_Save.insert(x_trackDatas_Save.begin(), x_trackDatas_Save[0]);
            y_trackDatas_Save.insert(y_trackDatas_Save.begin(), y_trackDatas_Save[0]);
            z_trackDatas_Save.insert(z_trackDatas_Save.begin(), z_trackDatas_Save[0]);
        }

        // Ӧ�ÿ������˲�
        std::vector<double> x_Filter = Cal_KalmanFilter(x_trackDatas_Save, MNoiseCov, PNoiseCov);
        std::vector<double> y_Filter = Cal_KalmanFilter(y_trackDatas_Save, MNoiseCov, PNoiseCov);
        std::vector<double> z_Filter = Cal_KalmanFilter(z_trackDatas_Save, MNoiseCov, PNoiseCov);

        return {
            x_Filter.empty() ? 0.0 : x_Filter.back(),
            y_Filter.empty() ? 0.0 : y_Filter.back(),
            z_Filter.empty() ? 0.0 : z_Filter.back()
        };
    }

    std::vector<double> TrackAlgMethod::Cal_KalmanFilter(
        const std::vector<double>& ilv_MeasureDatas,
        double idv_MNoiseCov,
        double idv_PNoiseCov)
    {
        if (ilv_MeasureDatas.empty()) {
            return {};
        }

        size_t n = ilv_MeasureDatas.size();
        std::vector<double> ldv_EstimationDatas(n, 0.0);
        double ldv_Gain = 0.0;
        double ldv_ProcessData = 10.0;  // ��ʼ����ֵ

        // ��ʼ����ֵ
        ldv_EstimationDatas[0] = ilv_MeasureDatas[0];

        for (size_t i = 1; i < n; ++i) {
            // Ԥ�ⲽ��
            ldv_EstimationDatas[i] = ldv_EstimationDatas[i - 1];
            ldv_ProcessData += idv_PNoiseCov;

            // ���²���
            ldv_Gain = ldv_ProcessData / (ldv_ProcessData + idv_MNoiseCov);
            ldv_EstimationDatas[i] += ldv_Gain * (ilv_MeasureDatas[i] - ldv_EstimationDatas[i]);
            ldv_ProcessData = (1.0 - ldv_Gain) * ldv_ProcessData;
        }

        return ldv_EstimationDatas;
    }

} // namespace WeldTrackApp