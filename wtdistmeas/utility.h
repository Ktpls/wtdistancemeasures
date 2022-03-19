#pragma once

#define TraverseX(sz) for (int x = 0; x < (sz).width; x++)
#define TraverseY(sz) for (int y = 0; y < (sz).height; y++)
#define TraverseXY(sz) TraverseX(sz) TraverseY(sz)
#define TraverseYX(sz) TraverseY(sz) TraverseX(sz)
#define SizeOfMat(m2d) Size((m2d).cols, (m2d).rows)
inline bool isKBDown(int k)
{
	return(GetAsyncKeyState(k) & 0x1);
}

inline bool isKBDownNow(int k)
{
	return(GetAsyncKeyState(k) & 0x8000);
}
void RGBA2RGB(const Mat& rgba, Mat& ret);