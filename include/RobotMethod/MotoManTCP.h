// include/RobotMethod/MotoManTCP.h
#pragma once

#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <random>
#include <cstring>
#include <stdexcept>
#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

// ������ͨ����ؽṹ�嶨��
struct RobotSendMsg {
    int count = 0;
    int serialNumber = 0;
    int ifRequestData = 0;
    int ifSetIOvalue = 0;
    int outputIOvalue[8] = { 0 };
    int ifStopTcp = 0;
    int CRCData = 0;

    // �˶�����
    struct IncDataType {
        int incData[6] = { 0 };
        int incDataType = 0;
        int toolNo = 0;
    };

    static constexpr int IncData_Count = 10;
    IncDataType datalist[IncData_Count];
};

struct RobotRecvMsg {
    double CartesianPos[6] = { 0 };
    double FLPCartesianPos[6] = { 0 };
    int inputIOvalue[8] = { 0 };
    int CurQueueCount = 0;
    int is_CRCOk = 0;
    int serialNumber = 0;
    int CRCData = 0;
};

struct RobotStatus {
    double CartesianPos[6] = { 0 };
    double FLPCartesianPos[6] = { 0 };
    int inputIOvalue[8] = { 0 };
    int CurQueueCount = 0;
};

// ͨ��״̬ö��
enum class Tcp_Comm_Status {
    tcpClient_null,
    tcpClient_no_connect,
    tcpClient_sendTimeOut,
    tcpClient_ok
};

enum class Tcp_Data_Status {
    tcpData_ok,
    tcpData_Recv_CRC_Error,
    tcpData_Send_CRC_Error,
    tcpData_Serial_Error,
    tcpData_null
};

class MotoManTCP {
public:
    // ��������
    static MotoManTCP& GetInstance() {
        static MotoManTCP instance;
        return instance;
    }

    // ��ֹ�����͸�ֵ
    MotoManTCP(const MotoManTCP&) = delete;
    MotoManTCP& operator=(const MotoManTCP&) = delete;

    // ���������������Ϣ
    void Gen_SendMsgToOut(int port, int value) {
        std::lock_guard<std::mutex> lock(queue_mutex_);

        RobotSendMsg msg;
        Init_RobotSendMsg(msg);

        if (port < 8) {
            msg.outputIOvalue[port] = value;
        }

        msg.ifSetIOvalue = 1;
        Cal_SendMsgCRC(msg);

        all_robot_send_msgs_.push_back(msg);
    }

    // �����˶�������Ϣ
    void Gen_SendMsgToQueue(const std::vector<std::vector<int>>& incDatas) {
        std::lock_guard<std::mutex> lock(queue_mutex_);

        const int incDataPerMsg = RobotSendMsg::IncData_Count;
        const int times = (incDatas.size() + incDataPerMsg - 1) / incDataPerMsg;

        for (int i = 0; i < times; i++) {
            RobotSendMsg msg;
            Init_RobotSendMsg(msg);

            int startIdx = i * incDataPerMsg;
            int count = 0;

            for (int j = 0; j < incDataPerMsg; j++) {
                int idx = startIdx + j;
                if (idx >= incDatas.size()) break;

                const auto& data = incDatas[idx];
                if (data.size() < 6) continue;

                std::memcpy(msg.datalist[j].incData, data.data(), 6 * sizeof(int));
                msg.datalist[j].incDataType = 0x90;
                msg.datalist[j].toolNo = tool_no_; // ʹ�����Ա����
                count++;
            }

            msg.count = count;
            Cal_SendMsgCRC(msg);
            all_robot_send_msgs_.push_back(msg);
        }
    }

    // ��ȡ������״̬
    RobotStatus GetRobotStatus() const {
        std::lock_guard<std::mutex> lock(status_mutex_);
        return robot_status_;
    }

    // ��ȡͨ��״̬
    Tcp_Comm_Status GetCommStatus() const {
        return comm_status_.load();
    }

    // ��ȡ����״̬
    Tcp_Data_Status GetDataStatus() const {
        return data_status_.load();
    }

    // ֹͣͨ��
    void Stop() {
        running_ = false;

        if (tcp_thread_.joinable()) {
            tcp_thread_.join();
        }

        CloseConnection();
        WSACleanup();
    }

private:
    // ��������
    static constexpr int tool_no_ = 1; // ���ߺ�
    static constexpr int queue_max_count_ = 50; // ��������
    static constexpr int crc_array_size_ = 128; // CRC���������С

    // ˽�й��캯��������ģʽ��
    MotoManTCP()
        : running_(true),
        comm_status_(Tcp_Comm_Status::tcpClient_no_connect),
        data_status_(Tcp_Data_Status::tcpData_null) {

        // ��ʼ��Winsock
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            throw std::runtime_error("WSAStartup failed");
        }

        // ����TCP
        if (TCP_Connect()) {
            tcp_thread_ = std::thread(&MotoManTCP::Do_TcpProcess, this);
        }
    }

    // ��ʼ��������Ϣ
    void Init_RobotSendMsg(RobotSendMsg& msg) {
        msg = RobotSendMsg(); // ����ΪĬ��ֵ
        msg.serialNumber = dist_(rng_);
        msg.ifRequestData = 1;
    }

    // ���㷢����Ϣ��CRC
    void Cal_SendMsgCRC(RobotSendMsg& msg) {
        // ���ṹ��ת��Ϊ�ֽ��������CRC����
        char buffer[sizeof(RobotSendMsg)];
        std::memcpy(buffer, &msg, sizeof(RobotSendMsg));

        // ����CRC����ʵ�֣�
        msg.CRCData = cal_crc(reinterpret_cast<int*>(buffer), sizeof(RobotSendMsg) / sizeof(int));
    }

    // TCPͨ���߳�
    void Do_TcpProcess() {
        RobotSendMsg current_send;
        RobotRecvMsg current_recv;
        char recv_buffer[sizeof(RobotRecvMsg)];

        while (running_) {
            // ׼��������Ϣ
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);

                if (!all_robot_send_msgs_.empty() &&
                    robot_status_.CurQueueCount < queue_max_count_) {

                    current_send = all_robot_send_msgs_.front();
                    all_robot_send_msgs_.pop_front();
                }
                else {
                    Init_RobotSendMsg(current_send);
                    Cal_SendMsgCRC(current_send);
                }
            }

            // ��������
            SendData(current_send);

            // ��������
            int wait_count = 0;
            bool received = false;

            while (running_ && !received) {
                fd_set read_set;
                FD_ZERO(&read_set);
                FD_SET(socket_, &read_set);

                timeval timeout = { 0, 10000 }; // 10ms

                // ����Ƿ������ݿɶ�
                int ready = select(0, &read_set, nullptr, nullptr, &timeout);
                if (ready > 0 && FD_ISSET(socket_, &read_set)) {
                    int bytes_received = recv(socket_, recv_buffer, sizeof(recv_buffer), 0);
                    if (bytes_received == sizeof(RobotRecvMsg)) {
                        std::memcpy(&current_recv, recv_buffer, sizeof(RobotRecvMsg));
                        received = true;
                    }
                }

                // ÿ10���ط�һ�Σ����Է�������Ϣ��
                if (++wait_count % 10 == 0 && current_send.count == 0) {
                    SendData(current_send);
                }

                // ��ʱ����
                if (wait_count > 100) {
                    data_status_ = Tcp_Data_Status::tcpData_null;
                    break;
                }
            }

            // ������յ�������
            if (received) {
                // ��֤CRC
                char recv_crc_buffer[sizeof(RobotRecvMsg)];
                std::memcpy(recv_crc_buffer, &current_recv, sizeof(RobotRecvMsg));
                int calculated_crc = cal_crc(reinterpret_cast<int*>(recv_crc_buffer),
                    sizeof(RobotRecvMsg) / sizeof(int));

                if (calculated_crc != current_recv.CRCData) {
                    data_status_ = Tcp_Data_Status::tcpData_Recv_CRC_Error;
                }
                else if (current_recv.is_CRCOk == 0) {
                    data_status_ = Tcp_Data_Status::tcpData_Send_CRC_Error;
                }
                else {
                    data_status_ = Tcp_Data_Status::tcpData_ok;

                    // ���»�����״̬
                    std::lock_guard<std::mutex> lock(status_mutex_);
                    for (int i = 0; i < 6; i++) {
                        robot_status_.CartesianPos[i] = current_recv.CartesianPos[i] / 1000.0;
                        robot_status_.FLPCartesianPos[i] = current_recv.FLPCartesianPos[i] / 1000.0;
                    }

                    for (int i = 0; i < 8; i++) {
                        robot_status_.inputIOvalue[i] = current_recv.inputIOvalue[i];
                    }

                    robot_status_.CurQueueCount = current_recv.CurQueueCount;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // ���ӵ������˿�����
    bool TCP_Connect() {
        SOCKADDR_IN server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(50240);
        inet_pton(AF_INET, "192.168.255.101", &server_addr.sin_addr);

        socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (socket_ == INVALID_SOCKET) {
            comm_status_ = Tcp_Comm_Status::tcpClient_null;
            return false;
        }

        // ���÷��ͳ�ʱ
        DWORD timeout = 100; // 100ms
        setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO,
            reinterpret_cast<const char*>(&timeout), sizeof(timeout));

        // ���ӷ�����
        if (connect(socket_, reinterpret_cast<SOCKADDR*>(&server_addr), sizeof(server_addr)) == SOCKET_ERROR) {
            closesocket(socket_);
            socket_ = INVALID_SOCKET;
            comm_status_ = Tcp_Comm_Status::tcpClient_no_connect;
            return false;
        }

        comm_status_ = Tcp_Comm_Status::tcpClient_ok;
        return true;
    }

    // �ر�����
    void CloseConnection() {
        if (socket_ != INVALID_SOCKET) {
            // ����ֹͣ����
            RobotSendMsg stop_msg;
            Init_RobotSendMsg(stop_msg);
            stop_msg.ifStopTcp = 1;
            Cal_SendMsgCRC(stop_msg);

            SendData(stop_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            SendData(stop_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // �ر��׽���
            closesocket(socket_);
            socket_ = INVALID_SOCKET;
        }
    }

    // ��������
    void SendData(const RobotSendMsg& msg) {
        if (socket_ == INVALID_SOCKET) {
            comm_status_ = Tcp_Comm_Status::tcpClient_null;
            return;
        }

        char send_buffer[sizeof(RobotSendMsg)];
        std::memcpy(send_buffer, &msg, sizeof(RobotSendMsg));

        if (send(socket_, send_buffer, sizeof(send_buffer), 0) == SOCKET_ERROR) {
            comm_status_ = Tcp_Comm_Status::tcpClient_sendTimeOut;
        }
        else {
            comm_status_ = Tcp_Comm_Status::tcpClient_ok;
        }
    }

    // ����CRCУ����
    int cal_crc(int* data, int length) {
        int crc = 0xffff;

        for (int i = 0; i < length; i++) {
            crc ^= data[i];

            for (int j = 0; j < 8; j++) {
                int flag = crc & 0x01;
                crc >>= 1;

                if (flag) {
                    crc ^= 0xA001;
                }
            }
        }

        return crc;
    }

private:
    // �������
    SOCKET socket_ = INVALID_SOCKET;

    // ���ݶ���
    std::deque<RobotSendMsg> all_robot_send_msgs_;
    std::mutex queue_mutex_;

    // ������״̬
    RobotStatus robot_status_;
    std::mutex status_mutex_;

    // ͨ��״̬
    std::atomic<Tcp_Comm_Status> comm_status_;
    std::atomic<Tcp_Data_Status> data_status_;

    // �߳̿���
    std::thread tcp_thread_;
    std::atomic<bool> running_;

    // ���������
    std::mt19937 rng_{std::random_device{}()};
    std::uniform_int_distribution<int> dist_{0, 10000};
};