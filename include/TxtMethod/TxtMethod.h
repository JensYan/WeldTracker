#pragma once

#include <vector>
#include <string>
#include <fstream>

class TxtMethod {
public:
	/**
	 * ���ı��ļ���ȡ���ݣ�ÿ�а���3��doubleֵ���ո�ָ���
	 * @param FileName �ļ�·��
	 * @return ��ά�������ڲ������̶�Ϊ3��Ԫ�� [x, y, z]
	 */
	std::vector<std::vector<double>> ReadData(const std::string& FileName);

	/**
	*������д���ı��ļ�����ֵ��ʽ��Ϊ"0000.00"��
	* @param ilv_LaserPoints ��д������ݣ���� = �У��ڲ� = �У�
	* @param FileName ����ļ�·��
	*/
	void WriteTxt(const std::vector<std::vector<double>>& ilv_LaserPoints,
		const std::string& FileName);
};