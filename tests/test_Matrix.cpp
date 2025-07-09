// MatrixTest.cpp
#include "gtest/gtest.h"
#include "ImageMethod/Matrix.h"

// 测试默认构造函数
TEST(MatrixTest, DefaultConstructor) {
    Matrix<int> mat;
    EXPECT_EQ(mat.Rows(), 1);
    EXPECT_EQ(mat.Columns(), 1);
    EXPECT_EQ(mat(0, 0), 0);
}

// 测试指定维度构造函数
TEST(MatrixTest, DimensionConstructor) {
    Matrix<double> mat(3, 4);
    EXPECT_EQ(mat.Rows(), 3);
    EXPECT_EQ(mat.Columns(), 4);
    
    // 验证所有元素初始化为0
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            EXPECT_DOUBLE_EQ(mat(i, j), 0.0);
        }
    }
}

// 测试二维向量构造函数
TEST(MatrixTest, Vector2DConstructor) {
    std::vector<std::vector<double>> data = {
        {1.0, 2.0, 3.0},
        {4.0, 5.0, 6.0}
    };
    
    Matrix<double> mat(data);
    EXPECT_EQ(mat.Rows(), 2);
    EXPECT_EQ(mat.Columns(), 3);
    
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            EXPECT_DOUBLE_EQ(mat(i, j), data[i][j]);
        }
    }
}

// 测试一维向量构造函数
TEST(MatrixTest, Vector1DConstructor) {
    std::vector<int> data = {1, 2, 3, 4, 5, 6};
    
    // 指定行列
    Matrix<int> mat1(2, 3, data);
    EXPECT_EQ(mat1.Rows(), 2);
    EXPECT_EQ(mat1.Columns(), 3);
    EXPECT_EQ(mat1(0, 0), 1);
    EXPECT_EQ(mat1(0, 1), 2);
    EXPECT_EQ(mat1(0, 2), 3);
    EXPECT_EQ(mat1(1, 0), 4);
    EXPECT_EQ(mat1(1, 1), 5);
    EXPECT_EQ(mat1(1, 2), 6);
    
    // 异常测试
    EXPECT_THROW(Matrix<int> mat2(2, data), std::invalid_argument);

    // 方阵
    std::vector<int> data2 = { 1, 2, 3, 4 };
    Matrix<int> mat2(2, data2);
    EXPECT_EQ(mat2.Rows(), 2);
    EXPECT_EQ(mat2.Columns(), 2);
    EXPECT_EQ(mat2(0, 0), 1);
    EXPECT_EQ(mat2(0, 1), 2);
    EXPECT_EQ(mat2(1, 0), 3);
    EXPECT_EQ(mat2(1, 1), 4);
}

// 测试元素访问和修改
TEST(MatrixTest, ElementAccess) {
    Matrix<double> mat(2, 2);
    
    // 测试操作符()
    mat(0, 0) = 1.5;
    mat(0, 1) = 2.5;
    mat(1, 0) = 3.5;
    mat(1, 1) = 4.5;
    
    EXPECT_DOUBLE_EQ(mat(0, 0), 1.5);
    EXPECT_DOUBLE_EQ(mat(0, 1), 2.5);
    EXPECT_DOUBLE_EQ(mat(1, 0), 3.5);
    EXPECT_DOUBLE_EQ(mat(1, 1), 4.5);
    
    // 测试GetElement/SetElement
    mat.SetElement(0, 0, 10.0);
    EXPECT_DOUBLE_EQ(mat.GetElement(0, 0), 10.0);
    
    // 越界访问应抛出异常
    EXPECT_THROW(mat(2, 0), std::out_of_range);
    EXPECT_THROW(mat(0, 2), std::out_of_range);
    EXPECT_THROW(mat.GetElement(-1, 0), std::out_of_range);
}

// 测试矩阵加法
TEST(MatrixTest, Addition) {
    Matrix<int> mat1({{1, 2}, {3, 4}});
    Matrix<int> mat2({{5, 6}, {7, 8}});
    
    Matrix<int> result = mat1 + mat2;
    
    EXPECT_EQ(result(0, 0), 6);
    EXPECT_EQ(result(0, 1), 8);
    EXPECT_EQ(result(1, 0), 10);
    EXPECT_EQ(result(1, 1), 12);
    
    // 维度不匹配应抛出异常
    Matrix<int> mat3(1, 1);
    EXPECT_THROW(mat1 + mat3, std::invalid_argument);
}

// 测试矩阵减法
TEST(MatrixTest, Subtraction) {
    Matrix<int> mat1({{5, 6}, {7, 8}});
    Matrix<int> mat2({{1, 2}, {3, 4}});
    
    Matrix<int> result = mat1 - mat2;
    
    EXPECT_EQ(result(0, 0), 4);
    EXPECT_EQ(result(0, 1), 4);
    EXPECT_EQ(result(1, 0), 4);
    EXPECT_EQ(result(1, 1), 4);
}

// 测试矩阵乘法
TEST(MatrixTest, Multiplication) {
    Matrix<int> mat1({{1, 2}, {3, 4}});
    Matrix<int> mat2({{2, 0}, {1, 2}});
    
    Matrix<int> result = mat1 * mat2;
    
    EXPECT_EQ(result(0, 0), 4);  // 1*2 + 2*1 = 4
    EXPECT_EQ(result(0, 1), 4);  // 1*0 + 2*2 = 4
    EXPECT_EQ(result(1, 0), 10); // 3*2 + 4*1 = 10
    EXPECT_EQ(result(1, 1), 8);  // 3*0 + 4*2 = 8
    
    // 维度不匹配应抛出异常
    Matrix<int> mat3(3, 3);
    EXPECT_THROW(mat1 * mat3, std::invalid_argument);
}

// 测试标量乘法
TEST(MatrixTest, ScalarMultiplication) {
    Matrix<int> mat({{1, 2}, {3, 4}});
    Matrix<int> result = mat * 3;
    
    EXPECT_EQ(result(0, 0), 3);
    EXPECT_EQ(result(0, 1), 6);
    EXPECT_EQ(result(1, 0), 9);
    EXPECT_EQ(result(1, 1), 12);
}

// 测试转置
TEST(MatrixTest, Transpose) {
    Matrix<int> mat({{1, 2, 3}, {4, 5, 6}});
    Matrix<int> trans = mat.Transpose();
    
    EXPECT_EQ(trans.Rows(), 3);
    EXPECT_EQ(trans.Columns(), 2);
    
    EXPECT_EQ(trans(0, 0), 1);
    EXPECT_EQ(trans(0, 1), 4);
    EXPECT_EQ(trans(1, 0), 2);
    EXPECT_EQ(trans(1, 1), 5);
    EXPECT_EQ(trans(2, 0), 3);
    EXPECT_EQ(trans(2, 1), 6);
}

// 测试单位矩阵
TEST(MatrixTest, UnitMatrix) {
    Matrix<double> mat;
    mat.MakeUnitMatrix(3);
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (i == j) {
                EXPECT_DOUBLE_EQ(mat(i, j), 1.0);
            } else {
                EXPECT_DOUBLE_EQ(mat(i, j), 0.0);
            }
        }
    }
}

// 测试矩阵求逆
TEST(MatrixTest, Inversion) {
    Matrix<double> mat({{4, 7}, {2, 6}});
    bool success = mat.InvertGaussJordan();
    
    EXPECT_TRUE(success);
    
    // 验证逆矩阵
    Matrix<double> identity = mat * Matrix<double>({{4, 7}, {2, 6}});
    
    // 检查是否近似单位矩阵
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            if (i == j) {
                EXPECT_NEAR(identity(i, j), 1.0, 1e-6);
            } else {
                EXPECT_NEAR(identity(i, j), 0.0, 1e-6);
            }
        }
    }
    
    // 测试奇异矩阵
    Matrix<double> singular({{1, 2}, {2, 4}});
    EXPECT_FALSE(singular.InvertGaussJordan());
    
    // 测试非方阵
    Matrix<double> nonSquare(2, 3);
    EXPECT_THROW(nonSquare.InvertGaussJordan(), std::invalid_argument);
}

// 测试行列式计算
TEST(MatrixTest, Determinant) {
    Matrix<double> mat({{1, 2}, {3, 4}});
    double det = mat.ComputeDetGauss();
    EXPECT_NEAR(det, -2.0, 1e-6);
    
    Matrix<double> mat2({{2, -1, 0}, {-1, 2, -1}, {0, -1, 2}});
    det = mat2.ComputeDetGauss();
    EXPECT_NEAR(det, 4.0, 1e-6);
    
    // 奇异矩阵的行列式应为0
    Matrix<double> singular({{1, 2}, {2, 4}});
    det = singular.ComputeDetGauss();
    EXPECT_NEAR(det, 0.0, 1e-6);
    
    // 非方阵应抛出异常
    Matrix<double> nonSquare(2, 3);
    EXPECT_THROW(nonSquare.ComputeDetGauss(), std::invalid_argument);
}

// 测试字符串表示
TEST(MatrixTest, ToString) {
    // double类型测试
    Matrix<double> mat({{1, 2}, {3, 4}});
    
    // 默认格式
    std::string str = mat.ToString();
    EXPECT_EQ(str, "1.000000,2.000000\n3.000000,4.000000");
    
    // 自定义分隔符和换行
    str = mat.ToString(";", false);
    EXPECT_EQ(str, "1.000000;2.000000;3.000000;4.000000");
    
    // 单行输出
    str = mat.ToStringRow(0, ",");
    EXPECT_EQ(str, "1.000000,2.000000");
    
    // 单列输出
    str = mat.ToStringCol(1, ",");
    EXPECT_EQ(str, "2.000000,4.000000");

    // int类型测试
    Matrix<int> mat2({ {1, 2}, {3, 4} });

    // 默认格式
    str = mat2.ToString();
    EXPECT_EQ(str, "1,2\n3,4");

    // 自定义分隔符和换行
    str = mat2.ToString(";", false);
    EXPECT_EQ(str, "1;2;3;4");

    // 单行输出
    str = mat2.ToStringRow(0, ",");
    EXPECT_EQ(str, "1,2");
    str = mat2.ToStringRow(0, ";");
    EXPECT_EQ(str, "1;2");

    // 单列输出
    str = mat2.ToStringCol(1, ",");
    EXPECT_EQ(str, "2,4");
    str = mat2.ToStringCol(1, " ");
    EXPECT_EQ(str, "2 4");
   
}

// 测试内存管理
TEST(MatrixTest, MemoryManagement) {
    // 测试赋值操作
    Matrix<int> mat1({{1, 2}, {3, 4}});
    Matrix<int> mat2 = mat1; // 拷贝构造
    
    EXPECT_EQ(mat2(0, 0), 1);
    EXPECT_EQ(mat2(0, 1), 2);
    
    // 保证是深拷贝
    mat1(0, 0) = 10;
    EXPECT_EQ(mat2(0, 0), 1);
    
    // 移动语义
    Matrix<int> mat3 = std::move(mat1);
    EXPECT_EQ(mat3(0, 0), 10);
    EXPECT_EQ(mat1.Rows(), 0); // 移动后原对象应为空
    EXPECT_EQ(mat1.GetData().size(), 0);
}

// 测试SetData和GetData
TEST(MatrixTest, SetGetData) {
    Matrix<int> mat(2, 2);
    std::vector<int> data = {1, 2, 3, 4};
    
    mat.SetData(data);
    
    std::vector<int> retrieved = mat.GetData();
    EXPECT_EQ(retrieved, data);
    
    // 数据大小不匹配应抛出异常
    std::vector<int> invalidData = {1, 2, 3};
    EXPECT_THROW(mat.SetData(invalidData), std::invalid_argument);
}

// LaserCoordToTcp.cpp中Matrix使用测试
TEST(MatrixTest, UseInLaserCoordToTcp) {
    // LaserCoordToTcp::Cal_TCPTranMat()测试
    Matrix<double> Tran_matrix(4, 4);
    // 需要确定初始值是0
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            EXPECT_EQ(Tran_matrix(i, j), 0);
        }
    }
}