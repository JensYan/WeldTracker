#pragma once

#include <vector>
#include <string>
#include <fstream>

class TxtMethod {
public:
	/**
	 * 从文本文件读取数据（每行包含3个double值，空格分隔）
	 * @param FileName 文件路径
	 * @return 二维向量，内层向量固定为3个元素 [x, y, z]
	 */
	std::vector<std::vector<double>> ReadData(const std::string& FileName);

	/**
	*将数据写入文本文件（数值格式化为"0000.00"）
	* @param ilv_LaserPoints 待写入的数据（外层 = 行，内层 = 列）
	* @param FileName 输出文件路径
	*/
	void WriteTxt(const std::vector<std::vector<double>>& ilv_LaserPoints,
		const std::string& FileName);
};