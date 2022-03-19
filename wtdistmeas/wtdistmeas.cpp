
#include "stdafx.h"
#include "wtdistmeas.h"
#include "utility.h"

void printmatd(const Mat& img)
{
    assert(img.type() == CV_64F);
    TraverseY(SizeOfMat(img))
    {
        TraverseX(SizeOfMat(img))
            printf("%12.3lf", img.at<double>(y, x));
        printf("\n");
    }
}

void showimg(const Mat& img, double f)
{
    Mat ishow;
    resize(img, ishow, Size(0, 0), f, f);
    namedWindow("", WINDOW_NORMAL);
    imshow("img", ishow);
    waitKey(0);
}

string yellowmarkpath = R"(yellowmark.png)";

void saveresult_withname(const Mat& img, const string& outputpath, const string& name, int& index)
{
    char buf[256];
    imwrite(outputpath + _itoa(index++, buf, 10) + "_" + name + ".png", img);
}

void pic2kernel(const Mat& iorg, Mat& mat)
{
    assert(iorg.type() == CV_8UC3);

    auto size = SizeOfMat(iorg);
    double total = 0, total2 = 0;
    size_t pix_eff = 0;

    //get transparent info
    Mat maque = Mat::zeros(SizeOfMat(iorg), CV_8U);
#pragma omp parallel for
    TraverseXY(size)
    {
        auto& pix = iorg.at<Vec3b>(y, x);
        //aque if non gray
        if (pix[0] != pix[1] || pix[0] != pix[2])
            maque.at<uchar>(y, x) = 1;
    }

    //to gray
    Mat igray;
    cvtColor(iorg, igray, COLOR_BGR2GRAY);

    //get general info
    TraverseXY(size)
    {
        if (maque.at<uchar>(y, x))
            continue;
        total += igray.at<uchar>(y, x);
        total2 += pow(igray.at<uchar>(y, x), 2);
        pix_eff++;
    }
    total /= pix_eff;
    total2 = sqrt(total2);

    //set mat
    mat = Mat::zeros(size, CV_64F);
#pragma omp parallel for
    TraverseXY(size)
    {
        mat.at<double>(y, x) = (maque.at<uchar>(y, x)) ? 0 : (igray.at<uchar>(y, x) - total) / total2;
    }
}



void RGB2HS_mine(const Vec3b& rgb, Vec2d& HS)
{
    static double arrayA[] = {
        1.0,cos(M_PI * 2 / 3),cos(M_PI * 4 / 3),
        0.0,sin(M_PI * 2 / 3),sin(M_PI * 4 / 3)
    };
    static const Mat A(2, 3, CV_64F, arrayA);

    Mat R = 1.0 / 255 * Mat(3, 1, CV_64F, rgb);

    Mat h = A * R;
    Vec2d vh(h);
    HS = Vec2d({
        180 / M_PI * atan2(vh[1], vh[0]),
        norm(vh)
        });
}

void rgb2hsv(int r, int g, int b, double& h, double& s, double& v)
{
    int imax, imin, diff;

    imax = r > g ? r : g;
    if (b > imax) imax = b;
    imin = r < g ? r : g;
    if (b < imin) imin = b;

    diff = imax - imin;
    v = imax * 100.0 / 255;
    //v = imax;
    if (imax == 0)
        s = 0;
    else
        s = diff * 100.0 / 255;

    if (diff != 0)
    {
        if (r == imax)
        {
            h = 60 * (g - b) / diff;
        }
        else if (g == imax)
        {
            h = 60 * (b - r) / diff + 120;
        }
        else
        {
            h = 60 * (r - g) / diff + 240;
        }

        if (h < 0)
            h = h + 360;
    }
    else
        h = 0;
}

Vec3d rgb2hsv(const Vec3b& rgb)
{
    Vec3d ret;
    rgb2hsv(rgb[2], rgb[1], rgb[0], ret[0], ret[1], ret[2]);
    return ret;
}

template<typename T>
T identity(const T&)
{
    return 0;
}

template <>
Point identity(const Point&)
{
    return Point(0, 0);
}
template <>
Point2d identity(const Point2d&)
{
    return Point2d(0, 0);
}


template<typename T>
T vecaverange(const vector<T>& a)
{
    T total;
    total = identity(total);
    for (const auto& i : a)
        total += i;
    return (T)(total / (double)a.size());
}

template<typename T, typename errT>
errT vecerror(const vector<T>& a, const T& averange, errT(*errfoo)(const T& diff))
{
    errT total;
    total = identity(total);
    for (const auto& i : a)
        total += errfoo(i - averange);
    return total / (double)a.size();
}

double vec3dcostheta(const Vec3d& a, const Vec3d& b)
{
    double normp = norm(a) * norm(b);
    if (normp <= DBL_EPSILON)
        return 0;
    return a.dot(b) / normp;
}

bool leastSquareLinearFit(float x[], float y[], const int num, float& a, float& b)
{
    if (num < 2)
        return false;
    float sum_x2 = 0.0;
    float sum_y = 0.0;
    float sum_x = 0.0;
    float sum_xy = 0.0;

    for (int i = 0; i < num; ++i) {
        sum_x2 += x[i] * x[i];
        sum_y += y[i];
        sum_x += x[i];
        sum_xy += x[i] * y[i];
    }

    a = (num * sum_xy - sum_x * sum_y) / (num * sum_x2 - sum_x * sum_x);
    b = (sum_x2 * sum_y - sum_x * sum_xy) / (num * sum_x2 - sum_x * sum_x);


    return true;
}

//ret OK for success
string SolveMap_BottomRightSmallMap(
    const Mat& isrc,
    double& errplayer,
    double& erryellowmark,
    double& errgrid,
    double& dist,
    bool dbg,
    const string& dbglogpath)
{
    int outputindex = 0;
    //sub pix
    Mat imap_colored;
    Point2i pa(1548, 729), pb(1871, 1052);
    getRectSubPix(isrc, Size(pb - pa), Point2f(0.5 * (pa + pb)), imap_colored);
    auto imgsize = SizeOfMat(imap_colored);

    if (dbg) saveresult_withname(imap_colored, dbglogpath, "subpix", outputindex);
    Mat imap;
    cvtColor(imap_colored, imap, COLOR_BGR2GRAY);
    if (dbg) saveresult_withname(imap, dbglogpath, "gray", outputindex);

    auto printf_redirected2file = [dbglogpath](const char* fmt, ...)
    {
        const size_t buffsize = 256;
        va_list args;
        va_start(args, fmt);
        char buf[buffsize];
        vsprintf(buf, fmt, args);
        va_end(args);
        ofstream of(dbglogpath + "log.log", ios::app);
        of.write(buf, strlen(buf));
    };

    //find player
    Point playerpos(0, 0);
    Mat iplayer = Mat::zeros(imgsize, CV_8U);
    {
        auto traceprintcontours = [&printf_redirected2file](vector<vector<cv::Point>> contours)
        {
            for (int i = 0; i < contours.size(); i++) {
                Point ave(0, 0);
                for (int j = 0; j < contours[i].size(); j++) {
                    ave += contours[i][j];
                }
                ave.x /= (int)contours[i].size();
                ave.y /= (int)contours[i].size();
                printf_redirected2file("contour%d@(%d,%d)S%f\n", i, ave.x, ave.y, contourArea(contours[i]));
            }
        };

        Mat ifilter = Mat::zeros(imgsize, CV_8U);
        Mat imap_colored_blured;
        GaussianBlur(imap_colored, imap_colored_blured, Size(5, 5), 1);
        if (dbg) saveresult_withname(imap_colored_blured, dbglogpath, "imap_colored_blured", outputindex);

#pragma omp parallel for
        TraverseXY(imgsize)
        {
            auto& pix = imap_colored_blured.at<Vec3b>(y, x);
            Vec3d HSV;
            rgb2hsv(pix[2], pix[1], pix[0], HSV[0], HSV[1], HSV[2]);
            if (HSV[0] >= 25 && HSV[0] <= 65)
                if (HSV[1] >= 15 && HSV[1] <= 60)
                    if (HSV[2] >= 55)
                        ifilter.at<uchar>(y, x) = 255;
        }
        if (dbg) saveresult_withname(ifilter, dbglogpath, "player_hsv_filter", outputindex);

        Mat iplayerthreshold;
        adaptiveThreshold(imap, iplayerthreshold, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 11, -110);
        if (dbg) saveresult_withname(iplayerthreshold, dbglogpath, "playeradaptiveThreshold", outputindex);
#pragma omp parallel for
        TraverseXY(imgsize)
        {
            if (ifilter.at<uchar>(y, x) != 255)
                iplayerthreshold.at<uchar>(y, x) = 0;
        }
        if (dbg) saveresult_withname(iplayerthreshold, dbglogpath, "player_hsv_filtered", outputindex);

        //contour
        vector<vector<cv::Point>> contours;
        findContours(iplayerthreshold, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        if (dbg)
        {
            Mat icontour = Mat::zeros(imgsize, CV_8U);
            drawContours(icontour, contours, -1, 255);
            printf_redirected2file("player all contours\n");
            traceprintcontours(contours);
            saveresult_withname(icontour, dbglogpath, "player all contours", outputindex);
        }

        //player arrow area should be between 8~15 according to experience
        vector<vector<cv::Point>> contoursAreaValid(contours.size());
        auto it = copy_if(begin(contours), end(contours), begin(contoursAreaValid),
            [](const vector<cv::Point>& i)
            {
                double area = contourArea(i);
                return (area >= 3.5 && area <= 16.5);
            });
        contoursAreaValid.resize(distance(begin(contoursAreaValid), it));  // shrink container to new size

        if (dbg)
        {
            drawContours(iplayer, contoursAreaValid, -1, 255, CV_FILLED);
            printf_redirected2file("player valid contours\n");
            traceprintcontours(contoursAreaValid);
            saveresult_withname(iplayer, dbglogpath, "iplayer", outputindex);
        }
        //assert only one area satisfied
        if (contoursAreaValid.size() != 1)
        {
            string errormsg = "PLY_NO_VALID_CONTOUR";
            printf_redirected2file(errormsg.c_str());
            return errormsg;
        }

        playerpos = vecaverange<Point>(contoursAreaValid[0]);
        errplayer = vecerror<Point, double>(contoursAreaValid[0], playerpos, [](const Point& i)->double {return pow(norm(i), 2.0); });

    }



    //find yellow mark
    Point yellowmarkpos(0, 0);
    Mat iyellowmark= imap.clone();
    {
        Mat ifilter = Mat::zeros(imgsize, CV_8U);
#pragma omp parallel for
        TraverseXY(imgsize)
        {
            auto& pix = imap_colored.at<Vec3b>(y, x);
            Vec3d HSV;
            rgb2hsv(pix[2], pix[1], pix[0], HSV[0], HSV[1], HSV[2]);
            if (HSV[0] >= 55 && HSV[0] <= 65)
                if (HSV[1] >= 45)
                    if (HSV[2] >= 20)
                        ifilter.at<uchar>(y, x) = 255;
        }
        if (dbg) saveresult_withname(ifilter, dbglogpath, "yellowmark_hsv_filter", outputindex);

#pragma omp parallel for
        TraverseXY(imgsize)
        {
            if (ifilter.at<uchar>(y, x) != 255)
                iyellowmark.at<uchar>(y, x) = 0;
        }
        if (dbg) saveresult_withname(iyellowmark, dbglogpath, "yellowmark_hsv_filtered", outputindex);

        //load yellow mark kernel
        Mat ikernel_yellowmark = imread(yellowmarkpath);
        Mat mkernel_yellowmark;
        pic2kernel(ikernel_yellowmark, mkernel_yellowmark);

        filter2D(iyellowmark, iyellowmark, -1, mkernel_yellowmark);
        if (dbg) saveresult_withname(iyellowmark, dbglogpath, "yellowmark_filter2D", outputindex);

        uchar max = 0;
        Point pmax(-1, -1);
        TraverseXY(imgsize)
        {
            if (iyellowmark.at<uchar>(y, x) > max)
            {
                pmax.x = x;
                pmax.y = y;
                max = iyellowmark.at<uchar>(y, x);
            }
        }
        //not found
        if (pmax.x < 0)
        {
            string errormsg = "YMR_NOTFOUND";
            printf_redirected2file(errormsg.c_str());
            return errormsg;
        }
        iyellowmark = Mat::zeros(imgsize, CV_8U);
        iyellowmark.at<uchar>(pmax) = 255;
        if (dbg) saveresult_withname(iyellowmark, dbglogpath, "iyellowmark", outputindex);
        yellowmarkpos = pmax;
        erryellowmark = 0;
    }

    //find grid
    double gridinterval = 0;
    Mat igrid = Mat::zeros(imgsize, CV_8U);
    {
        Mat igridproc = Mat::zeros(imgsize, CV_8U);
        //negative
#pragma omp parallel for
        TraverseXY(imgsize)
            igridproc.at<uchar>(y, x) = 255 - imap.at<uchar>(y, x);
        if (dbg) saveresult_withname(igridproc, dbglogpath, "reversed", outputindex);

        //adaptiveThreshold
        //brighter area
        adaptiveThreshold(igridproc, igridproc, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 5, -5);
        if (dbg) saveresult_withname(igridproc, dbglogpath, "adaptiveThreshold", outputindex);

        //get line

        //generate kernel
        const size_t gridlinekernellength = 201;
        double akernel[3 * gridlinekernellength];
        for (size_t i = 0; i < gridlinekernellength; i++)
        {

            akernel[i] = -0.5;
            akernel[i + gridlinekernellength] = 1;
            akernel[i + 2 * gridlinekernellength] = -0.5;
        }
        Mat kernelrow = Mat(3, gridlinekernellength, CV_64F, akernel) / gridlinekernellength;
        Mat kernelcol = kernelrow.t();

        Mat icol;
        filter2D(igridproc, icol, -1, kernelcol);

        Mat irow;
        filter2D(igridproc, irow, -1, kernelrow);

        //make together
        igridproc = irow + icol;
        if (dbg) saveresult_withname(igridproc, dbglogpath, "grid_filtered", outputindex);

        threshold(igridproc, igridproc, 70, 0, THRESH_TOZERO);
        if (dbg) saveresult_withname(igridproc, dbglogpath, "Threshold_grid", outputindex);

        adaptiveThreshold(igridproc, igridproc, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 5, -50);
        if (dbg) saveresult_withname(igridproc, dbglogpath, "adaptiveThreshold_grid", outputindex);

        //grid reduction
        //statistics
        vector<int> rowstatistic(igridproc.rows), colstatistic(igridproc.cols);
        TraverseXY(imgsize)
        {
            if (igridproc.at<uchar>(y, x) == 255)
            {
                rowstatistic[y]++;
                colstatistic[x]++;
            }
        }

        //to binary
        for (auto& i : rowstatistic)
            i = (double)i / igridproc.cols > 0.25 ? 255 : 0;
        for (auto& i : colstatistic)
            i = (double)i / igridproc.rows > 0.25 ? 255 : 0;

        if (dbg)
        {
            //view
#pragma omp parallel for
            TraverseXY(imgsize)
            {
                if (rowstatistic[y] > 0 || colstatistic[x] > 0)
                    igrid.at<uchar>(y, x) = 255;
            }
            saveresult_withname(igrid, dbglogpath, "igrid", outputindex);
        }
        auto getGridInterval = [](const vector<int>& stt, vector<double>& intervals)->bool
        {
            vector<int> linepos;
            for (size_t i = 0; i < stt.size(); i++)
                if (stt[i] > 0)
                    linepos.push_back((double)i);
            if (linepos.size() < 2)
                return false;

            intervals.resize(linepos.size() - 1);
            for (size_t i = 0; i < intervals.size(); i++)
                intervals[i] = linepos[i + 1] - linepos[i];
            return true;
        };

        auto printintervals = [printf_redirected2file](const vector<double>& intervals)
        {
            for (auto i : intervals)
            {
                printf_redirected2file("%lf\n", i);
            }
        };

        double ave, err;

        vector<double> intervaltmp, intervaltotal;

        if (!getGridInterval(colstatistic, intervaltmp))
        {
            string errormsg = "GRD_INTERVAL_COL";
            printf_redirected2file(errormsg.c_str());
            return errormsg;
        }
        intervaltotal.insert(end(intervaltotal), begin(intervaltmp), end(intervaltmp));
        if (!getGridInterval(rowstatistic, intervaltmp))
        {
            string errormsg = "GRD_INTERVAL_ROW";
            printf_redirected2file(errormsg.c_str());
            return errormsg;
        }
        intervaltotal.insert(end(intervaltotal), begin(intervaltmp), end(intervaltmp));
        printf_redirected2file("all intervals\n");
        printintervals(intervaltotal);

        //filter intervals
#define MIN_LINE_INTERVAL 20
        auto it = copy_if(begin(intervaltotal), end(intervaltotal), begin(intervaltotal), [](double i) {return (i > MIN_LINE_INTERVAL); });
        intervaltotal.resize(distance(begin(intervaltotal), it));  // shrink container to new size

        size_t samplenumtotal = intervaltotal.size();
        while (true)
        {
            ave = vecaverange<double>(intervaltotal);
            err = vecerror<double, double>(intervaltotal, ave, [](const double& i)->double {return (double)(i * i); });
            if (err < 10)
            {
                if (dbg) printf_redirected2file("grid interval samples used %d out of %d, err=%f\n", intervaltotal.size(), samplenumtotal, err);
                break;
            }
            if (intervaltotal.size() < 2)
            {
                string errormsg = "GRD_INTERVAL_AVE";
                printf_redirected2file(errormsg.c_str());
                return errormsg;
            }
            vector<double> errs(intervaltotal.size());
            for (size_t i = 0; i < errs.size(); i++)
            {
                errs[i] = pow(intervaltotal[i] - ave, 2);
            }
            intervaltotal.erase(begin(intervaltotal) + distance(begin(errs), max_element(begin(errs), end(errs))));
        }

        printf_redirected2file("used intervals\n");
        printintervals(intervaltotal);

        errgrid = err;
        gridinterval = ave;
    }
    dist = norm(yellowmarkpos - playerpos) / gridinterval;
    if (dbg)
    {
        saveresult_withname(iplayer + iyellowmark + igrid, dbglogpath, "result", outputindex);
        printf_redirected2file("player=(%5d,%5d)\n", playerpos.x, playerpos.y);
        printf_redirected2file("yellowmark=(%5d,%5d)\n", yellowmarkpos.x, yellowmarkpos.y);
        printf_redirected2file("gridinterval=%5f\n", gridinterval);
    }
    return "OK";
}
