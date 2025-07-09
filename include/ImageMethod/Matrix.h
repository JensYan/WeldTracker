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
    // ���캯��
    Matrix() : numRows(1), numColumns(1), elements(1) {}
    Matrix(int nRows, int nCols) : numRows(nRows), numColumns(nCols), elements(nRows* nCols) {}
    Matrix(int nSize) : numRows(nSize), numColumns(nSize), elements(nSize* nSize) {}
    Matrix(const std::vector<std::vector<T>>& value);
    Matrix(int nRows, int nCols, const std::vector<T>& value);
    Matrix(int nSize, const std::vector<T>& value);

    // �������캯�� -> ���
    Matrix(const Matrix& other) = default;
    // �ƶ����캯��
    Matrix(Matrix&& other) noexcept;

    // ������ֵ
    Matrix& operator=(const Matrix& other) = default;
    // �ƶ���ֵ 
    Matrix& operator=(Matrix&& other) noexcept;

    // ���������
    T& operator()(int row, int col);
    const T& operator()(int row, int col) const;
    explicit operator std::vector<T>() const { return elements; }

    Matrix operator+(const Matrix& other) const;
    Matrix operator-(const Matrix& other) const;
    Matrix operator*(const Matrix& other) const;
    Matrix operator*(T value) const;

    // �������ԺͲ���
    int Rows() const { return numRows; }
    int Columns() const { return numColumns; }
    T Eps() const { return eps; }
    void SetEps(T newEps) { eps = newEps; }

    void Init(int nRows, int nCols);
    void SetData(const std::vector<T>& data);
    bool SetElement(int row, int col, T value);
    T GetElement(int row, int col) const;
    std::vector<T> GetData() const { return elements; }

    // �������
    bool MakeUnitMatrix(int nSize);
    Matrix Transpose() const;

    // ���Դ�������
    bool InvertGaussJordan();
    T ComputeDetGauss() const;

    // �ַ�����ʾ
    std::string ToString() const;
    std::string ToString(const std::string& sDelim, bool bLineBreak) const;
    std::string ToStringRow(int nRow, const std::string& sDelim) const;
    std::string ToStringCol(int nCol, const std::string& sDelim) const;

private:
    int numRows;
    int numColumns;
    std::vector<T> elements;
    T eps = static_cast<T>(1e-12);

    // �ڲ���������
    void ppp(std::vector<T>& a, const std::vector<T>& e, const std::vector<T>& s,
        std::vector<T>& v, int m, int n) const;
    void sss(std::vector<T>& fg, std::vector<T>& cs) const;
};

// ʵ��ģ���������ͷ�ļ���
#include "Matrix.inl"
