#pragma once

#include <cstdint>
#include <array>

namespace WeldTrackApp {

	namespace MacroDefine {
        constexpr int RobotRecvMsg_CRC_Count = 36;        // �ɽṹ�� RobotRecvMsg Լ��
        constexpr int RobotSendMsg_CRC_Count = 115;       // �ɽṹ�� RobotSendMsg Լ��
        constexpr int IncData_Count = 10;                 // �ṹ�� RobotSendMsg �е����·��� IncData ����
        constexpr int Motoman_Queue_MCount = 300;         // Motoman �������ݶ����е������·����ݵ����ֵ
        constexpr int toolNo = 15;                        // ��ǰ��ǹ����ϵ��ֵ���� Motoman ʾ����ȷ��
        constexpr double WeldSpeed = 15.0;                // �趨�����ٶ�Ϊ��λ mm/s
        constexpr double InterCycle = 0.01;               // Motoman �岹����Ϊ 10ms
        constexpr int filterDelay = 180;                  // �������˲����ӳ�ʱ�� 200
        constexpr double Cab_Corr_X = 0.00;               // �궨��������λ mm
        constexpr double Cab_Corr_Y = 0.00;
        constexpr double Cab_Corr_Z = 0.0;
}

    enum class TcpCommStatus {
        tcpClient_ok,
        tcpClient_null,
        tcpClient_no_connect,
        tcpClient_sendTimeOut
};

    enum class TcpDataStatus {
        tcpData_ok,
        tcpData_null,
        tcpData_Serial_Error,
        tcpData_Send_CRC_Error,
        tcpData_Recv_CRC_Error
    };

    enum class CameraStatus {
        Camera_ok,
        Camera_Connect_Err,
        Camera_Data_Timeout,
        Camera_GrabFailed,
        Camera_Running_Err
    };

    enum class TrackStatus {
        Track_null,
        Send_StartPt,
        Wait_ArriveStartPt,
        Track_Runing,
        Wait_Track_Stop,
        Sim_Start,
        Sim_Run
    };

    struct RobotRecvMsg {
        int32_t serialNumber = 0;

        std::array<int32_t, 8> PulsePos = {};
        std::array<int32_t, 8> CartesianPos = {};
        std::array<int32_t, 8> FLPCartesianPos = {};
        std::array<int32_t, 8> inputIOvalue = {};

        int32_t CurQueueCount = 0;
        int32_t is_CRCOk = 0;              // ��ʶPC�˴���������CRC�Ƿ���ȷ����ȷΪ1������Ϊ0 
        int32_t CRCData = 0;
    };

    struct RobotRecvMsg_CRC {
        std::array<int32_t, MacroDefine::RobotRecvMsg_CRC_Count> RobotRecvMsg_CRC_Value = {};
    };

    struct IncData {
        std::array<int32_t, 8> incData = {};
        int32_t toolNo = MacroDefine::toolNo; // ʹ�ó�����ʼ��
        int32_t incDataType = 0;              // ������������
    };

    struct RobotSendMsg {
        int32_t serialNumber = 0;          // ���к�

        std::array<IncData, MacroDefine::IncData_Count> datalist = {};
        std::array<int32_t, 8> outputIOvalue = {};   // ��Ӧ�˿�IOֵ

        int32_t count = 0;                 // ��Ǵ˴��·��ĵ���
        int32_t ifclear = 0;               // 0��ʾ��ն��У�1��ʾ�������
        int32_t ifRequestData = 0;         // 0��ʾ���������ݣ�1��ʾ��������
        int32_t ifStopTcp = 0;             // 0��ʾ�������ӣ�1��ʾ�˳�����
        int32_t ifSetIOvalue = 0;          // 0��ʾ���������������ֵ��1��ʾ�������������ֵ 
        int32_t CRCData = 0;               // CRCУ����  

    };

    struct RobotSendMsg_CRC {
        std::array<int32_t, MacroDefine::RobotSendMsg_CRC_Count> RobotSendMsg_CRC_Value = {};
    };

    struct RobotStatus {
        std::array<double, 8> CartesianPos = {};
        std::array<double, 8> FLPCartesianPos = {};
        std::array<int32_t, 8> inputIOvalue = {};
        int32_t CurQueueCount = 0;
    };
}   // namesapce WeldTrackApp