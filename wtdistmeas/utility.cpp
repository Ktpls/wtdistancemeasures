
#include "stdafx.h"
#include "utility.h"
void RGBA2RGB(const Mat& rgba, Mat& ret)
{
	assert(rgba.type() == CV_8UC4);
	auto size = SizeOfMat(rgba);
	ret = Mat::zeros(size, CV_8UC3);
	TraverseXY(size)
	{
		auto& l = ret.at<Vec3b>(y, x);
		auto& r = rgba.at<Vec4b>(y, x);
		for (size_t i = 0; i < 3; i++)
			l[(int)i] = r[(int)i];
	}
}