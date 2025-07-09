// include/ImageMethod/HGetImage.h
#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>

// Halcon ͷ�ļ�
#include "HalconCpp.h"

// Pylon ��� SDK ͷ�ļ�
#include <pylon/PylonIncludes.h>

namespace fs = std::filesystem;
using namespace HalconCpp;
using namespace Pylon;

class HGetImage {
public:
    // ��������
    static HGetImage& GetInstance() {
        static HGetImage instance;
        return instance;
    }

    // ��ֹ�����͸�ֵ
    HGetImage(const HGetImage&) = delete;
    HGetImage& operator=(const HGetImage&) = delete;

    // �����
    bool Open() {
        std::lock_guard<std::mutex> lock(camera_mutex_);

        try {
            if (camera_) {
                return true; // ����Ѵ�
            }

            // �����������
            camera_ = std::make_unique<CInstantCamera>(CTlFactory::GetInstance().CreateFirstDevice());

            // ע�������¼�
            camera_->RegisterConfiguration(new CAcquireContinuousConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);

            // ���ò���
            camera_->MaxNumBuffer = 5;

            // ע��ͼ��ץȡ�ص�
            camera_->RegisterImageEventHandler(this, RegistrationMode_Append, Cleanup_None);

            // �����
            camera_->Open();

            // ��ʼץȡ
            camera_->StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByInstantCamera);

            return true;
        }
        catch (const GenericException& e) {
            last_error_ = e.GetDescription();
            camera_.reset();
            return false;
        }
    }

    // ֹͣ���
    void Stop() {
        std::lock_guard<std::mutex> lock(camera_mutex_);

        try {
            if (camera_ && camera_->IsGrabbing()) {
                camera_->StopGrabbing();
            }
            if (camera_ && camera_->IsOpen()) {
                camera_->Close();
            }
            camera_.reset();
        }
        catch (...) {
            // ����ֹͣ�����еĴ���
        }
    }

    // ��ȡ��ǰͼ��
    HImage GetImage(bool save_image = false) {
        std::lock_guard<std::mutex> lock(image_mutex_);

        if (save_image) {
            SaveCurrentImage();
        }

        return current_image_;
    }

    // ���ļ���ȡͼ�񣨲����ã�
    HImage GetImageFromFile() {
        std::lock_guard<std::mutex> lock(file_mutex_);

        if (track_files_.empty()) {
            LoadImageFiles();
        }

        if (frame_num_ < track_files_.size()) {
            try {
                current_image_.ReadImage(track_files_[frame_num_].c_str());
                frame_num_++;
            }
            catch (HException& e) {
                last_error_ = e.ErrorMessage().Text();
            }
        }

        return current_image_;
    }

    // ��ȡ��������Ϣ
    std::string GetLastError() const {
        return last_error_;
    }

    // Pylon ͼ���¼�����ӿ�
    void OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& grabResult) {
        if (!grabResult->GrabSucceeded()) {
            last_error_ = "Image grab failed";
            return;
        }

        try {
            // ����ͼ�񻥳���
            std::lock_guard<std::mutex> lock(image_mutex_);

            // ����Halconͼ��
            const uint8_t* buffer = static_cast<const uint8_t*>(grabResult->GetBuffer());
            int width = static_cast<int>(grabResult->GetWidth());
            int height = static_cast<int>(grabResult->GetHeight());

            // ������ͼ�����
            current_image_.GenImage1("byte", width, height, const_cast<HBYTE*>(buffer));
        }
        catch (HException& e) {
            last_error_ = e.ErrorMessage().Text();
        }
    }

private:
    // ˽�й��캯��������ģʽ��
    HGetImage()
        : frame_path_("E:/yan/WeldTrackApp_lcy/test-L/"),
        frame_num_(0),
        if_save_(false) {
        try {
            // ������ͼ��
            current_image_.GenImageConst("byte", 1, 1);
        }
        catch (HException& e) {
            last_error_ = e.ErrorMessage().Text();
        }
    }

    // ���浱ǰͼ��
    void SaveCurrentImage() {
        try {
            // ȷ��Ŀ¼����
            fs::create_directories(frame_path_);

            // �����ļ���
            std::string filename = frame_path_ + std::to_string(frame_num_) + ".jpg";

            // ����ͼ��
            current_image_.WriteImage("jpeg", 0, filename.c_str());

            // ����֡����
            frame_num_++;
        }
        catch (const std::exception& e) {
            last_error_ = e.what();
        }
    }

    // ����ͼ���ļ��б�
    void LoadImageFiles() {
        try {
            std::string test_path = "E:/yan/WeldTrackApp_lcy/test_3/";

            if (!fs::exists(test_path)) {
                last_error_ = "Test image directory not found";
                return;
            }

            // �ռ�����JPG�ļ�
            for (const auto& entry : fs::directory_iterator(test_path)) {
                if (entry.path().extension() == ".jpg") {
                    track_files_.push_back(entry.path().string());
                }
            }

            // ���޸�ʱ������
            std::sort(track_files_.begin(), track_files_.end(),
                [](const std::string& a, const std::string& b) {
                    return fs::last_write_time(a) < fs::last_write_time(b);
                });
        }
        catch (const std::exception& e) {
            last_error_ = e.what();
        }
    }

private:
    // �������
    std::unique_ptr<CInstantCamera> camera_;

    // ��ǰͼ��
    HImage current_image_;

    // �ļ�·��
    std::string frame_path_;
    std::vector<std::string> track_files_;

    // ״̬����
    int frame_num_;
    bool if_save_;
    std::string last_error_;

    // ������
    std::mutex camera_mutex_;
    std::mutex image_mutex_;
    std::mutex file_mutex_;
};