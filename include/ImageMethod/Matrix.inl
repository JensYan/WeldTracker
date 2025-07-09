// 三种构造函数实现
template <typename T>
Matrix<T>::Matrix(const std::vector<std::vector<T>>& value)
    : numRows(value.size()), numColumns(value.empty() ? 0 : value[0].size())
{
    elements.resize(numRows * numColumns);
    for (int i = 0; i < numRows; ++i) {
        for (int j = 0; j < numColumns; ++j) {
            elements[i * numColumns + j] = value[i][j];
        }
    }
}
template <typename T>
Matrix<T>::Matrix(int nRows, int nCols, const std::vector<T>& value)
    : numRows(nRows), numColumns(nCols)
{
    if (static_cast<int>(value.size()) != nRows * nCols) {
        throw std::invalid_argument("Input data size does not match matrix dimensions");
    }
    elements = value;
}
template <typename T>
Matrix<T>::Matrix(int nSize, const std::vector<T>& value)
    : numRows(nSize), numColumns(nSize)
{
    if (static_cast<int>(value.size()) != nSize * nSize) {
        throw std::invalid_argument("Input data size does not match matrix dimensions");
    }
    elements = value;
}

// 移动构造函数
template <typename T>
Matrix<T>::Matrix(Matrix&& other) noexcept
    : numRows(other.numRows),          // 直接转移行数
    numColumns(other.numColumns),    // 直接转移列数
    elements(std::move(other.elements))  // 使用 std::move 转移 vector
{
    // 置空源对象的状态
    other.numRows = 0;
    other.numColumns = 0;
    // 注意：other.elements 已经被移动，自动变为空 vector
}
// 移动赋值运算符
template <typename T>
Matrix<T>& Matrix<T>::operator=(Matrix&& other) noexcept {
    if (this != &other) {
        // 不需要手动释放资源！vector 会自动管理

        // 转移资源
        numRows = other.numRows;
        numColumns = other.numColumns;
        elements = std::move(other.elements);  // 使用 std::move

        // 置空源对象
        other.numRows = 0;
        other.numColumns = 0;
        // other.elements 已被移动，自动为空
    }
    return *this;
}

// 元素访问
template <typename T>
T& Matrix<T>::operator()(int row, int col) {
    if (row < 0 || row >= numRows || col < 0 || col >= numColumns) {
        throw std::out_of_range("Matrix indices out of range");
    }
    return elements[col + row * numColumns];
}

template <typename T>
const T& Matrix<T>::operator()(int row, int col) const {
    if (row < 0 || row >= numRows || col < 0 || col >= numColumns) {
        throw std::out_of_range("Matrix indices out of range");
    }
    return elements[col + row * numColumns];
}

// 矩阵运算
template <typename T>
Matrix<T> Matrix<T>::operator+(const Matrix& other) const {
    if (numColumns != other.numColumns || numRows != other.numRows) {
        throw std::invalid_argument("Matrix dimensions do not match for addition");
    }

    Matrix result(*this);
    for (int i = 0; i < numRows; ++i) {
        for (int j = 0; j < numColumns; ++j) {
            result(i, j) += other(i, j);
        }
    }
    return result;
}

template <typename T>
Matrix<T> Matrix<T>::operator-(const Matrix& other) const {
    if (numColumns != other.numColumns || numRows != other.numRows) {
        throw std::invalid_argument("Matrix dimensions do not match for subtraction");
    }

    Matrix result(*this);
    for (int i = 0; i < numRows; ++i) {
        for (int j = 0; j < numColumns; ++j) {
            result(i, j) -= other(i, j);
        }
    }
    return result;
}

template <typename T>
Matrix<T> Matrix<T>::operator*(const Matrix& other) const {
    if (numColumns != other.numRows) {
        throw std::invalid_argument("Matrix dimensions do not match for multiplication");
    }

    Matrix result(numRows, other.numColumns);
    for (int i = 0; i < result.numRows; ++i) {
        for (int j = 0; j < result.numColumns; ++j) {
            T sum = 0;
            for (int k = 0; k < numColumns; ++k) { 
                sum += (*this)(i, k) * other(k, j);
            }
            result(i, j) = sum;
        }
    }
    return result;
}

template <typename T>
Matrix<T> Matrix<T>::operator*(T value) const {
    Matrix result(*this);
    for (auto& elem : result.elements) {
        elem *= value;
    }
    return result;
}

// 矩阵操作
template <typename T>
void Matrix<T>::Init(int nRows, int nCols) {
    if (nRows < 0 || nCols < 0) {
        throw std::invalid_argument("Invalid matrix dimensions");
    }

    numRows = nRows;
    numColumns = nCols;
    elements.resize(nRows * nCols);
    std::fill(elements.begin(), elements.end(), T{});   // 这一行是什么意思？怎么管理内存
}

template <typename T>
void Matrix<T>::SetData(const std::vector<T>& data) {
    if (static_cast<int>(data.size()) != numRows * numColumns) {
        throw std::invalid_argument("Input data size does not match matrix dimensions");
    }
    //elements = data;    // 源代码 (double[])value.Clone() 这里似乎没有创建新内存给elements
    elements.assign(data.begin(), data.end());
}

template <typename T>
bool Matrix<T>::SetElement(int row, int col, T value) {
    if (row < 0 || row >= numRows || col < 0 || col >= numColumns) {
        return false;
    }
    elements[col + row * numColumns] = value;
    return true;
}

template <typename T>
T Matrix<T>::GetElement(int row, int col) const {
    if (row < 0 || row >= numRows || col < 0 || col >= numColumns) {
        throw std::out_of_range("Matrix indices out of range");
    }
    return elements[col + row * numColumns];
}

template <typename T>
bool Matrix<T>::MakeUnitMatrix(int nSize) {
    //if (!Init(nSize, nSize)) {      // C++的Init函数声明是void
    //    return false;
    //}
    Init(nSize, nSize);

    for (int i = 0; i < nSize; ++i) {
        SetElement(i, i, 1);
    }
    return true;
}

template <typename T>
Matrix<T> Matrix<T>::Transpose() const {
    Matrix trans(numColumns, numRows);
    for (int i = 0; i < numRows; ++i) {
        for (int j = 0; j < numColumns; ++j) {
            trans(j, i) = (*this)(i, j);
        }
    }
    return trans;   // 这里如何管理内存？旧的Matrix不删除内存吗？
}


// 线性代数运算
template <typename T>
bool Matrix<T>::InvertGaussJordan() {
    if (numRows != numColumns) {
        throw std::invalid_argument("Matrix must be square for inversion");
    }

    const int n = numRows;
    std::vector<int> pnRow(n), pnCol(n);

    for (int k = 0; k < n; k++) {
        T d = 0.0;
        for (int i = k; i < n; i++) {
            for (int j = k; j < n; j++) {
                T p = std::abs((*this)(i, j));
                if (p > d) {
                    d = p;
                    pnRow[k] = i;
                    pnCol[k] = j;
                }
            }
        }

        if (d == 0.0) return false;

        if (pnRow[k] != k) {
            for (int j = 0; j < n; j++) {
                std::swap((*this)(k, j), (*this)(pnRow[k], j));
            }
        }

        if (pnCol[k] != k) {
            for (int i = 0; i < n; i++) {
                std::swap((*this)(i, k), (*this)(i, pnCol[k]));
            }
        }

        (*this)(k, k) = 1.0 / (*this)(k, k);
        for (int j = 0; j < n; j++) {
            if (j != k) {
                (*this)(k, j) *= (*this)(k, k);
            }
        }

        for (int i = 0; i < n; i++) {
            if (i != k) {
                for (int j = 0; j < n; j++) {
                    if (j != k) {
                        (*this)(i, j) -= (*this)(i, k) * (*this)(k, j);
                    }
                }
                (*this)(i, k) = -(*this)(i, k) * (*this)(k, k);
            }
        }
    }

    for (int k = n - 1; k >= 0; k--) {
        if (pnCol[k] != k) {
            for (int j = 0; j < n; j++) {
                std::swap((*this)(k, j), (*this)(pnCol[k], j));
            }
        }
        if (pnRow[k] != k) {
            for (int i = 0; i < n; i++) {
                std::swap((*this)(i, k), (*this)(i, pnRow[k]));
            }
        }
    }

    return true;
}

// 行列式计算 (高斯消元法) -> 未校对代码
template <typename T>
T Matrix<T>::ComputeDetGauss() const {
    if (numRows != numColumns) {
        throw std::invalid_argument("Matrix must be square for determinant calculation");
    }

    const int n = numRows;
    Matrix<T> temp(*this);
    T det = 1.0;

    for (int k = 0; k < n - 1; k++) {
        // 部分主元选择
        int maxRow = k;
        T maxVal = std::abs(temp(k, k));
        for (int i = k + 1; i < n; i++) {
            T absVal = std::abs(temp(i, k));
            if (absVal > maxVal) {
                maxVal = absVal;
                maxRow = i;
            }
        }

        if (maxVal == 0.0) return 0.0;

        if (maxRow != k) {
            // 交换行
            for (int j = k; j < n; j++) {
                std::swap(temp(k, j), temp(maxRow, j));
            }
            det = -det; // 行交换改变行列式符号
        }

        // 高斯消元
        for (int i = k + 1; i < n; i++) {
            T factor = temp(i, k) / temp(k, k);
            for (int j = k + 1; j < n; j++) {
                temp(i, j) -= factor * temp(k, j);
            }
        }

        det *= temp(k, k);
    }

    det *= temp(n - 1, n - 1);
    return det;
}

// 字符串表示
template <typename T>
std::string Matrix<T>::ToString() const {
    return ToString(",", true);
}

template <typename T>
std::string Matrix<T>::ToString(const std::string& sDelim, bool bLineBreak) const {
    std::ostringstream oss;
    for (int i = 0; i < numRows; ++i) {
        for (int j = 0; j < numColumns; ++j) {
            oss << std::fixed << std::setprecision(6) << (*this)(i, j);
            if (bLineBreak) {
                if (j != numColumns - 1) oss << sDelim;
            }
            else {
                if (i != numRows - 1 || j != numColumns - 1) oss << sDelim;
            }
        }
        if (bLineBreak && i != numRows - 1) oss << "\n";
    }
    return oss.str();
}

template <typename T>
std::string Matrix<T>::ToStringRow(int nRow, const std::string& sDelim) const {
    if (nRow < 0 || nRow >= numRows) {
        return "";
    }

    std::ostringstream oss;
    for (int j = 0; j < numColumns; ++j) {
        oss << std::fixed << std::setprecision(6) << (*this)(nRow, j);
        if (j != numColumns - 1) oss << sDelim;
    }
    return oss.str();
}

template <typename T>
std::string Matrix<T>::ToStringCol(int nCol, const std::string& sDelim) const {
    if (nCol < 0 || nCol >= numColumns) {
        return "";
    }

    std::ostringstream oss;
    for (int i = 0; i < numRows; ++i) {
        oss << std::fixed << std::setprecision(6) << (*this)(i, nCol);
        if (i != numRows - 1) oss << sDelim;
    }
    return oss.str();
}

// 内部辅助函数
template <typename T>
void Matrix<T>::ppp(std::vector<T>& a, const std::vector<T>& e, const std::vector<T>& s,
    std::vector<T>& v, int m, int n) const
{
    T i, j, p, q;
    T d;

    if (m >= n)
        i = n;
    else
        i = m;

    for (j = 1; j <= i - 1; j++)
    {
        a[(j - 1) * n + j - 1] = s[j - 1];
        a[(j - 1) * n + j] = e[j - 1];
    }

    a[(i - 1) * n + i - 1] = s[i - 1];
    if (m < n)
        a[(i - 1) * n + i] = e[i - 1];

    for (i = 1; i <= n - 1; i++)
    {
        for (j = i + 1; j <= n; j++)
        {
            p = (i - 1) * n + j - 1;
            q = (j - 1) * n + i - 1;
            d = v[p];
            v[p] = v[q];
            v[q] = d;
        }
    }
}

template <typename T>
void Matrix<T>::sss(std::vector<T>& fg, std::vector<T>& cs) const
{
    T r, d;

    if ((std::abs(fg[0]) + std::abs(fg[1])) == 0.0)
    {
        cs[0] = 1.0;
        cs[1] = 0.0;
        d = 0.0;
    }
    else
    {
        d = std::sqrt(fg[0] * fg[0] + fg[1] * fg[1]);
        if (std::abs(fg[0]) > std::abs(fg[1]))
        {
            d = std::abs(d);
            if (fg[0] < 0.0)
                d = -d;
        }
        if (std::abs(fg[1]) >= std::abs(fg[0]))
        {
            d = std::abs(d);
            if (fg[1] < 0.0)
                d = -d;
        }

        cs[0] = fg[0] / d;
        cs[1] = fg[1] / d;
    }

    r = 1.0;
    if (std::abs(fg[0]) > std::abs(fg[1]))
        r = cs[1];
    else if (cs[0] != 0.0)
        r = 1.0 / cs[0];

    fg[0] = d;
    fg[1] = r;
}

