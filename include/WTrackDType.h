#pragma once

#include <cstdint>
#include <array>

namespace WeldTrackApp {

	namespace MacroDefine {
        constexpr int RobotRecvMsg_CRC_Count = 36;        // 由结构体 RobotRecvMsg 约束
        constexpr int RobotSendMsg_CRC_Count = 115;       // 由结构体 RobotSendMsg 约束
        constexpr int IncData_Count = 10;                 // 结构体 RobotSendMsg 中单次下发的 IncData 数量
        constexpr int Motoman_Queue_MCount = 300;         // Motoman 增量数据队列中的允许下发数据的最大值
        constexpr int toolNo = 15;                        // 当前焊枪坐标系的值，由 Motoman 示教器确定
        constexpr double WeldSpeed = 15.0;                // 设定焊接速度为单位 mm/s
        constexpr double InterCycle = 0.01;               // Motoman 插补周期为 10ms
        constexpr int filterDelay = 180;                  // 卡尔曼滤波器延迟时间 200
        constexpr double Cab_Corr_X = 0.00;               // 标定误差补偿，单位 mm
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
        int32_t is_CRCOk = 0;              // 标识PC端传来的数据CRC是否正确，正确为1，错误为0 
        int32_t CRCData = 0;
    };

    struct RobotRecvMsg_CRC {
        std::array<int32_t, MacroDefine::RobotRecvMsg_CRC_Count> RobotRecvMsg_CRC_Value = {};
    };

    struct IncData {
        std::array<int32_t, 8> incData = {};
        int32_t toolNo = MacroDefine::toolNo; // 使用常量初始化
        int32_t incDataType = 0;              // 增量数据类型
    };

    struct RobotSendMsg {
        int32_t serialNumber = 0;          // 序列号

        std::array<IncData, MacroDefine::IncData_Count> datalist = {};
        std::array<int32_t, 8> outputIOvalue = {};   // 对应端口IO值

        int32_t count = 0;                 // 标记此次下发的点数
        int32_t ifclear = 0;               // 0表示清空队列，1表示正常入队
        int32_t ifRequestData = 0;         // 0表示不请求数据，1表示请求数据
        int32_t ifStopTcp = 0;             // 0表示正常连接，1表示退出连接
        int32_t ifSetIOvalue = 0;          // 0表示不设置数字量输出值，1表示设置数字量输出值 
        int32_t CRCData = 0;               // CRC校验码  

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