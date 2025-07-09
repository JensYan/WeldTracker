#include "RobotMethod/TrackAlgMethod.h"
#include <gtest/gtest.h>
#include <vector>
#include <cmath>

using namespace WeldTrackApp;

// 测试夹具类
class TrackAlgMethodTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 创建测试轨迹数据
        trajectory_3points = {
            {1.0, 2.0, 3.0, 10.0, 20.0, 30.0},  // x,y,z,rx,ry,rz
            {4.0, 5.0, 6.0, 40.0, 50.0, 60.0},
            {7.0, 8.0, 9.0, 70.0, 80.0, 90.0}
        };
        
        trajectory_2points = {
            {1.0, 2.0, 3.0, 10.0, 20.0, 30.0},
            {4.0, 5.0, 6.0, 40.0, 50.0, 60.0}
        };
        
        // 创建长轨迹用于性能测试
        for (int i = 0; i < 1000; ++i) {
            long_trajectory.push_back({static_cast<double>(i), 
                                       static_cast<double>(i*2), 
                                       static_cast<double>(i*3),
                                       static_cast<double>(i), 
                                       static_cast<double>(i*2), 
                                       static_cast<double>(i*3)});
        }
    }
    
    TrackAlgMethod alg;
    std::vector<std::vector<double>> trajectory_3points;
    std::vector<std::vector<double>> trajectory_2points;
    std::vector<std::vector<double>> long_trajectory;
};

// 1. 测试角度增量计算
TEST_F(TrackAlgMethodTest, Cal_IncAtt_HandlesAngleWrap) {
    // 测试正常角度
    EXPECT_NEAR(alg.Cal_IncAtt(10.0, 20.0), 10.0, 1e-6);
    
    // 测试负角度（跨360°边界）
    EXPECT_NEAR(alg.Cal_IncAtt(-10.0, 10.0), 20.0, 1e-6);  // -10° -> 350°, 350°->10°=20°增量
    EXPECT_NEAR(alg.Cal_IncAtt(350.0, 10.0), 20.0, 1e-6);
    
    // 测试大角度变化
    EXPECT_NEAR(alg.Cal_IncAtt(-170.0, 170.0), -20.0, 1e-6);  // 实际应返回-340°，但算法处理后为20°
    
    // 测试相同角度
    EXPECT_NEAR(alg.Cal_IncAtt(30.0, 30.0), 0.0, 1e-6);
}

// 2. 测试距离计算
TEST_F(TrackAlgMethodTest, Cal_Length_CalculatesCorrectDistance) {
    std::vector<double> p1 = {0.0, 0.0, 0.0};
    std::vector<double> p2 = {3.0, 4.0, 0.0};
    EXPECT_NEAR(alg.Cal_Length(p1, p2), 5.0, 1e-6);
    
    std::vector<double> p3 = {-1.0, 2.0, 3.0};
    std::vector<double> p4 = {-4.0, 6.0, 9.0};
    EXPECT_NEAR(alg.Cal_Length(p3, p4), 7.81025, 1e-6);
    
    // 测试无效输入
    std::vector<double> invalid = {1.0};
    EXPECT_THROW(alg.Cal_Length(p1, invalid), std::invalid_argument);
}

// 3. 测试焊缝参数计算
TEST_F(TrackAlgMethodTest, Cal_WeldPara_CalculatesCorrectValues) {
    std::vector<double> incAtt;
    double totalLen;
    
    // 测试有效输入
    EXPECT_TRUE(alg.Cal_WeldPara(trajectory_3points, incAtt, totalLen));
    EXPECT_EQ(incAtt.size(), 3);
    EXPECT_NEAR(incAtt[0], 60.0, 1e-6);  // 10°->70°=60°
    EXPECT_NEAR(incAtt[1], 60.0, 1e-6);  // 20°->80°=60°
    EXPECT_NEAR(incAtt[2], 60.0, 1e-6);  // 30°->90°=60°
    EXPECT_NEAR(totalLen, 2 * std::sqrt(27.0), 1e-6);  // 两段各√(3²+3²+3²)=√27
    
    // 测试不足点的情况
    std::vector<std::vector<double>> single_point = {{1.0, 2.0, 3.0}};
    EXPECT_FALSE(alg.Cal_WeldPara(single_point, incAtt, totalLen));
    
    // 测试空输入
    std::vector<std::vector<double>> empty;
    EXPECT_FALSE(alg.Cal_WeldPara(empty, incAtt, totalLen));
}

// 4. 测试插补点生成
TEST_F(TrackAlgMethodTest, Cal_InterPt_GeneratesCorrectPoints) {
    std::vector<double> p1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> p2 = {150.0, 0.0, 0.0, 90.0, 0.0, 0.0};  // 100mm, 90°旋转
    
    // 计算插补点数量: 150mm / (15mm/s * 0.01s) = 150 / 0.15 = 1000点
    auto interPts = alg.Cal_InterPt(p1, p2, 300.0, {90.0, 0.0, 0.0});
    
    ASSERT_EQ(interPts.size(), 1000);
    
    // 检查位置增量 IncPos = static_cast<int>((p2 - p1) / IncNum * 1000)
    double inc = 150.0 / 1000.0 * 1000.0;
    int expectedPosInc = static_cast<int>(inc);  // 0.01mm = 10单位
    EXPECT_EQ(interPts[0][0], expectedPosInc);
    EXPECT_EQ(interPts[0][1], 0);
    EXPECT_EQ(interPts[0][2], 0);  // Z轴增量为0
    
    // IncAtt = static_cast<int>((TotalIncAtt * Len/TotalLen) / IncNum * 10000 )
    // 检查姿态增量: 90° / 100mm * 0.1mm / 10点 = 0.009°/点 = 90单位
    inc = 90.0 * 150 / (300.0 * 1000) * 10000;
    int expectedAttInc = static_cast<int>(inc);
    EXPECT_EQ(interPts[0][3], expectedAttInc);
    
    // 测试总长为0的情况
    EXPECT_THROW(alg.Cal_InterPt(p1, p2, 0.0, {90.0, 0.0, 0.0}), std::invalid_argument);

    // 测试两点长度大于总长的情况
    EXPECT_THROW(alg.Cal_InterPt(p1, p2, 100.0, { 90.0, 0.0, 0.0 }), std::invalid_argument);
    
    // 测试距离过短的情况（无插补点）
    p2[0] = 0.005;  // 0.005mm距离 -> 0.005/0.15=0.5点 → 0点
    auto noPoints = alg.Cal_InterPt(p1, p2, 100.0, {90.0, 0.0, 0.0});
    EXPECT_TRUE(noPoints.empty());

    // 测试合适距离的插值
    p2[0] = 1.5;    // 1.5 / 0.15 = 10点
    auto suitPts = alg.Cal_InterPt(p1, p2, 300.0, { 90.0, 0.0, 0.0 });
    ASSERT_EQ(suitPts.size(), 10);
    expectedPosInc = static_cast<int>(1.5 / 10 * 1000);
    EXPECT_EQ(suitPts[0][0], expectedPosInc);
    EXPECT_EQ(interPts[0][1], 0);
    EXPECT_EQ(interPts[0][2], 0);
    expectedAttInc = static_cast<int>(90.0 * 1.5 * 10000 / (300.0 * 10));
    EXPECT_EQ(interPts[0][3], expectedAttInc);

    // 测试姿态不变的插值
    auto zeroAttInc = alg.Cal_InterPt(p1, p2, 300.0, { 0.0, 0.0, 0.0 });
    inc = 0.0 * 1.5 * 10000 / (300.0 * 10);
    expectedAttInc = static_cast<int>(inc);
    EXPECT_EQ(zeroAttInc[0][3], expectedAttInc);
}

// 5. 测试总长度计算
TEST_F(TrackAlgMethodTest, Cal_totalLength_CalculatesCorrectLength) {
    // 创建特定轨迹数据
    std::vector<std::vector<double>> trackData = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},  // 第5-7元素为位置
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 4.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 8.0, 0.0}
    };
    
    double length = alg.Cal_totalLength(trackData);
    EXPECT_NEAR(length, 10.0, 1e-6);  // 5+5
    
    // 测试无效数据
    std::vector<std::vector<double>> invalidData = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0}  // 不足3个位置元素
    };
    EXPECT_THROW(alg.Cal_totalLength(invalidData), std::invalid_argument);
    invalidData = {
        {0.0, 0.0},
        {0.0, 0.0, 0.0}  // 不足3个位置元素
    };
    EXPECT_THROW(alg.Cal_totalLength(invalidData), std::invalid_argument);
    invalidData = {
        {0.0, 0.0},
        {0.0, 0.0}  // 不足3个位置元素
    };
    EXPECT_THROW(alg.Cal_totalLength(invalidData), std::invalid_argument);

    // 测试数据不足
    std::vector<std::vector<double>> shortData = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };
    EXPECT_NEAR(alg.Cal_totalLength(shortData), 0.0, 1e-6);
}

// 6. 测试插补决策
TEST_F(TrackAlgMethodTest, Inter_Decision_ReturnsCorrectResult) {
    // Len / (15 * 0.01) = （Len / 0.15） > 1 ? yes : no
    std::vector<double> p1 = {0.0, 0.0, 0.0};
    std::vector<double> p2_short = {0.005, 0.0, 0.0};  // 0.005mm
    std::vector<double> p2_long = {0.3, 0.0, 0.0};    // 0.02mm
    
    // 短距离: 0.005/(15*0.01)=0.005/0.15=0.333 → 0点 → false
    EXPECT_FALSE(alg.Inter_Decision(p1, p2_short));
    
    // 长距离: 0.3/(15*0.01)=0.3/0.15=2点 → true
    EXPECT_TRUE(alg.Inter_Decision(p1, p2_long));
}

// 7. 测试批量插补点生成
TEST_F(TrackAlgMethodTest, Gen_BatchTrackIncPtData_GeneratesCorrectBatch) {
    // 创建测试轨迹数据
    std::vector<std::vector<double>> trackData = {
        {0.0, 0.0, 0.0},
        {0.05, 0.0, 0.0},  // 短距离，不应生成插补点
        {0.3, 0.0, 0.0},   // 应生成插补点
        {0.6, 0.0, 0.0}    // 应生成插补点
    };
    
    auto originalData = trackData;  // 保存副本
    
    // 生成插补点
    auto incPoints = alg.Gen_BatchTrackIncPtData(trackData, 100.0, {90.0, 0.0, 0.0});
    
    // 验证结果
    // 应有2段生成插补点：0.0->0.3 (1段) 和 0.3->0.6 (1段)
    // 每段距离0.3mm → 2个插补点/段 → 共4个插补点
    EXPECT_EQ(incPoints.size(), 4);
    
    // 验证数据被正确移除
    // 原始数据: 4点 → 移除前3点，保留最后1点
    EXPECT_EQ(trackData.size(), 1);
    EXPECT_EQ(trackData[0], originalData.back());
}

// 8. 测试测量位置滤波
TEST_F(TrackAlgMethodTest, mea_Pos_Filter_FiltersPositionData) {
    // 创建带噪声的轨迹数据
    std::vector<std::vector<double>> noisyTrack;
    for (int i = 0; i < 5; ++i) {
        double noise = 0.1 * (std::rand() % 100 - 50) / 50.0;
        noisyTrack.push_back({10.0 + noise, 20.0 + noise, 30.0 + noise});
    }
    
    // 应用滤波
    double MNoiseCov = 0.1;  // 测量噪声协方差
    double PNoiseCov = 0.01; // 过程噪声协方差
    auto filtered = alg.mea_Pos_Filter(noisyTrack, MNoiseCov, PNoiseCov);
    
    // 验证结果
    ASSERT_EQ(filtered.size(), 3);
    
    // 滤波后的值应接近平均值
    double sumX = 0.0;
    for (const auto& point : noisyTrack) {
        sumX += point[0];
    }
    double avgX = sumX / noisyTrack.size();
    
    EXPECT_NEAR(filtered[0], avgX, 0.05);  // 允许5%误差
}

// 9. 性能测试
TEST_F(TrackAlgMethodTest, PerformanceTest) {
    // 测试Cal_WeldPara性能
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<double> incAtt;
    double totalLen;
    alg.Cal_WeldPara(long_trajectory, incAtt, totalLen);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    // 1000点应在1ms内完成（根据硬件调整）
    EXPECT_LT(duration.count(), 5);

    GTEST_LOG_(INFO) << "The speed of Cal_WeldPara(): " << duration.count() << " us";
    
    // 测试批量插补点生成性能
    auto trackData = long_trajectory;
    start = std::chrono::high_resolution_clock::now();
    alg.Gen_BatchTrackIncPtData(trackData, totalLen, incAtt);
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    // 1000点应在5ms内完成（根据硬件调整）
    EXPECT_LT(duration.count(), 5000);

    GTEST_LOG_(INFO) << "The speed of Gen_BatchTrackIncPtData(): " << duration.count() << " us";
}

// 10. 测试边界条件
TEST_F(TrackAlgMethodTest, BoundaryConditions) {
    // 测试极大值
    std::vector<double> p1 = {-1e6, -1e6, -1e6, -360.0, -360.0, -360.0};
    std::vector<double> p2 = {1e6, 1e6, 1e6, 360.0, 360.0, 360.0};
    
    std::vector<double> incAtt;
    double totalLen;
    EXPECT_NO_THROW(alg.Cal_WeldPara({p1, p2}, incAtt, totalLen));
}

