#include "TxtMethod.h"
#include <sstream>
#include <iomanip>
#include <iostream>

// ������������ʽ��doubleֵΪ"0000.00"��ʽ
static std::string FormatDouble(double value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << std::setw(7) << std::setfill('0') << value;
    return oss.str();
}

std::vector<std::vector<double>> TxtMethod::ReadData(const std::string& FileName) {
    std::vector<std::vector<double>> result;
    std::ifstream file(FileName);

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << FileName << std::endl;
        return result;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::vector<double> values;
        std::string token;

        // ��ȡ���3����ֵ
        while (iss >> token && values.size() < 3) {
            try {
                values.push_back(std::stod(token));
            }
            catch (...) {
                // ת��ʧ��ʱ������ǰ��
                values.clear();
                break;
            }
        }

        // ȷ����3����Чֵ
        if (values.size() == 3) {
            result.push_back(std::move(values));
        }
    }
    return result;
}

void TxtMethod::WriteTxt(const std::vector<std::vector<double>>& ilv_LaserPoints,
    const std::string& FileName) {
    std::ofstream file(FileName);
    if (!file.is_open()) {
        std::cerr << "Error creating file: " << FileName << std::endl;
        return;
    }

    for (const auto& point : ilv_LaserPoints) {
        std::string line;
        for (size_t j = 0; j < point.size(); ++j) {
            line += FormatDouble(point[j]);
            if (j < point.size() - 1) {
                line += " ";
            }
        }
        file << line << "\n";
    }
}