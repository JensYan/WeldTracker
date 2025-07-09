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

// Halcon 头文件
#include "HalconCpp.h"

// Pylon 相机 SDK 头文件
#include <pylon/PylonIncludes.h>

namespace fs = std::filesystem;
using namespace HalconCpp;
using namespace Pylon;

class HGetImage {
public:
    // 单例访问
    static HGetImage& GetInstance() {
        static HGetImage instance;
        return instance;
    }

    // 禁止拷贝和赋值
    HGetImage(const HGetImage&) = delete;
    HGetImage& operator=(const HGetImage&) = delete;

    // 打开相机
    bool Open() {
        std::lock_guard<std::mutex> lock(camera_mutex_);

        try {
            if (camera_) {
                return true; // 相机已打开
            }

            // 创建相机对象
            camera_ = std::make_unique<CInstantCamera>(CTlFactory::GetInstance().CreateFirstDevice());

            // 注册配置事件
            camera_->RegisterConfiguration(new CAcquireContinuousConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);

            // 设置参数
            camera_->MaxNumBuffer = 5;

            // 注册图像抓取回调
            camera_->RegisterImageEventHandler(this, RegistrationMode_Append, Cleanup_None);

            // 打开相机
            camera_->Open();

            // 开始抓取
            camera_->StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByInstantCamera);

            return true;
        }
        catch (const GenericException& e) {
            last_error_ = e.GetDescription();
            camera_.reset();
            return false;
        }
    }

    // 停止相机
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
            // 忽略停止过程中的错误
        }
    }

    // 获取当前图像
    HImage GetImage(bool save_image = false) {
        std::lock_guard<std::mutex> lock(image_mutex_);

        if (save_image) {
            SaveCurrentImage();
        }

        return current_image_;
    }

    // 从文件获取图像（测试用）
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

    // 获取最后错误信息
    std::string GetLastError() const {
        return last_error_;
    }

    // Pylon 图像事件处理接口
    void OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& grabResult) {
        if (!grabResult->GrabSucceeded()) {
            last_error_ = "Image grab failed";
            return;
        }

        try {
            // 锁定图像互斥锁
            std::lock_guard<std::mutex> lock(image_mutex_);

            // 创建Halcon图像
            const uint8_t* buffer = static_cast<const uint8_t*>(grabResult->GetBuffer());
            int width = static_cast<int>(grabResult->GetWidth());
            int height = static_cast<int>(grabResult->GetHeight());

            // 创建新图像对象
            current_image_.GenImage1("byte", width, height, const_cast<HBYTE*>(buffer));
        }
        catch (HException& e) {
            last_error_ = e.ErrorMessage().Text();
        }
    }

private:
    // 私有构造函数（单例模式）
    HGetImage()
        : frame_path_("E:/yan/WeldTrackApp_lcy/test-L/"),
        frame_num_(0),
        if_save_(false) {
        try {
            // 创建空图像
            current_image_.GenImageConst("byte", 1, 1);
        }
        catch (HException& e) {
            last_error_ = e.ErrorMessage().Text();
        }
    }

    // 保存当前图像
    void SaveCurrentImage() {
        try {
            // 确保目录存在
            fs::create_directories(frame_path_);

            // 生成文件名
            std::string filename = frame_path_ + std::to_string(frame_num_) + ".jpg";

            // 保存图像
            current_image_.WriteImage("jpeg", 0, filename.c_str());

            // 增加帧计数
            frame_num_++;
        }
        catch (const std::exception& e) {
            last_error_ = e.what();
        }
    }

    // 加载图像文件列表
    void LoadImageFiles() {
        try {
            std::string test_path = "E:/yan/WeldTrackApp_lcy/test_3/";

            if (!fs::exists(test_path)) {
                last_error_ = "Test image directory not found";
                return;
            }

            // 收集所有JPG文件
            for (const auto& entry : fs::directory_iterator(test_path)) {
                if (entry.path().extension() == ".jpg") {
                    track_files_.push_back(entry.path().string());
                }
            }

            // 按修改时间排序
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
    // 相机对象
    std::unique_ptr<CInstantCamera> camera_;

    // 当前图像
    HImage current_image_;

    // 文件路径
    std::string frame_path_;
    std::vector<std::string> track_files_;

    // 状态变量
    int frame_num_;
    bool if_save_;
    std::string last_error_;

    // 互斥锁
    std::mutex camera_mutex_;
    std::mutex image_mutex_;
    std::mutex file_mutex_;
};