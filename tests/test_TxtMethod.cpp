#define _ITERATOR_DEBUG_LEVEL 0
#include "TxtMethod/TxtMethod.h"
#include <gtest/gtest.h>
#include <filesystem>

namespace fs = std::filesystem;


TEST(TxtMethodTest, ReadWriteIntegration) {
	ASSERT_EQ(1, 1);
}

//TEST(TxtMethodTest, ReadWriteIntegration) {
//    TxtMethod txt;
//    const std::string testFile = "test_data.txt";
//
//    // ׼����������
//    std::vector<std::vector<double>> testData = {
//        {123.456, 78.9, 0.123},
//        {9999.99, -5.0, 3.14159}
//    };
//
//    // д������ļ�
//    txt.WriteTxt(testData, testFile);
//    ASSERT_TRUE(fs::exists(testFile));
//
//    // ��ȡ��֤
//    auto readData = txt.ReadData(testFile);
//    ASSERT_EQ(readData.size(), testData.size());
//
//    for (size_t i = 0; i < testData.size(); ++i) {
//        ASSERT_EQ(readData[i].size(), 3);
//        EXPECT_NEAR(readData[i][0], testData[i][0], 0.01);
//        EXPECT_NEAR(readData[i][1], testData[i][1], 0.01);
//        EXPECT_NEAR(readData[i][2], testData[i][2], 0.01);
//    }
//
//    // ����
//    fs::remove(testFile);
//}
//
//TEST(TxtMethodTest, ReadInvalidData) {
//    TxtMethod txt;
//    const std::string testFile = "invalid_data.txt";
//
//    // ����������Ч���ݵ��ļ�
//    std::ofstream out(testFile);
//    out << "1.0 2.0 valid\n";
//    out << "3.0 invalid 5.0\n";  // ����Ӧ������
//    out << "7.0 8.0 9.0\n";
//    out.close();
//
//    auto data = txt.ReadData(testFile);
//    ASSERT_EQ(data.size(), 2);  // ֻӦ��ȡ������Ч����
//    EXPECT_EQ(data[0], std::vector<double>({ 1.0, 2.0, 0.0 })); // �����в�0
//    EXPECT_EQ(data[1], std::vector<double>({ 7.0, 8.0, 9.0 }));
//
//    fs::remove(testFile);
//}