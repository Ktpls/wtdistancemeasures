#pragma once
void showimg(const Mat& img, double f = 1);
bool SolveMap(
    const Mat& isrc,
    double& errplayer,
    double& errcur,
    double& errgrid,
    double& dist,
    bool dbg = false);
string SolveMap_BottomRightSmallMap(
    const Mat& isrc,
    double& errplayer,
    double& erryellowmark,
    double& errgrid,
    double& dist,
    bool dbg = false,
    const string & dbglogpath=R"(C:\wtdistmeas\)");