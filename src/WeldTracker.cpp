#include <iostream>
#include "WTrackDType.h"
#include "ImageMethod/Matrix.h"
#include "TxtMethod/TxtMethod.h"

using namespace std;

int main()
{
	cout << "Hello CMake." << endl;
	WeldTrackApp::IncData incData;
	Matrix<double> m1;
	int a = m1.Rows();
	cout << a << endl;

	cout << "ok" << endl;
	return 0;
}
