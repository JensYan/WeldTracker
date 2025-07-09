#pragma once
#include <vector>
#include <cmath>
#include <stdexcept>
#include <iomanip>
#include <sstream>
#include <algorithm>
//#include <funcional>

template <typename T = double>
class Matrix
{
public:
    // 构造函数
    Matrix() : numRows(1), numColumns(1), elements(1) {}
    Matrix(int nRows, int nCols) : numRows(nRows), numColumns(nCols), elements(nRows* nCols) {}
    Matrix(int nSize) : numRows(nSize), numColumns(nSize), elements(nSize* nSize) {}
    Matrix(const std::vector<std::vector<T>>& value);
    Matrix(int nRows, int nCols, const std::vector<T>& value);
    Matrix(int nSize, const std::vector<T>& value);

    // 拷贝构造函数 -> 深拷贝
    Matrix(const Matrix& other) = default;
    // 移动构造函数
    Matrix(Matrix&& other) noexcept;

    // 拷贝赋值
    Matrix& operator=(const Matrix& other) = default;
    // 移动赋值 
    Matrix& operator=(Matrix&& other) noexcept;

    // 运算符重载
    T& operator()(int row, int col);
    const T& operator()(int row, int col) const;
    explicit operator std::vector<T>() const { return elements; }

    Matrix operator+(const Matrix& other) const;
    Matrix operator-(const Matrix& other) const;
    Matrix operator*(const Matrix& other) const;
    Matrix operator*(T value) const;

    // 基本属性和操作
    int Rows() const { return numRows; }
    int Columns() const { return numColumns; }
    T Eps() const { return eps; }
    void SetEps(T newEps) { eps = newEps; }

    void Init(int nRows, int nCols);
    void SetData(const std::vector<T>& data);
    bool SetElement(int row, int col, T value);
    T GetElement(int row, int col) const;
    std::vector<T> GetData() const { return elements; }

    // 矩阵操作
    bool MakeUnitMatrix(int nSize);
    Matrix Transpose() const;

    // 线性代数运算
    bool InvertGaussJordan();
    T ComputeDetGauss() const;

    // 字符串表示
    std::string ToString() const;
    std::string ToString(const std::string& sDelim, bool bLineBreak) const;
    std::string ToStringRow(int nRow, const std::string& sDelim) const;
    std::string ToStringCol(int nCol, const std::string& sDelim) const;

private:
    int numRows;
    int numColumns;
    std::vector<T> elements;
    T eps = static_cast<T>(1e-12);

    // 内部辅助函数
    void ppp(std::vector<T>& a, const std::vector<T>& e, const std::vector<T>& s,
        std::vector<T>& v, int m, int n) const;
    void sss(std::vector<T>& fg, std::vector<T>& cs) const;
};

// 实现模板类必须在头文件中
#include "Matrix.inl"
