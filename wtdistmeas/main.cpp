// wtdistmeas.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include "stdafx.h"
#include "utility.h"
#include "wtdistmeas.h"
#include "Screenshot.h"
#include "toast.h"

void solvemapfromdisk()
{
    Mat imap = imread(R"(D:\output\i\i.jpg)");
    double errplayer=-1, errmark = -1, errgrid = -1;
    double dist;
    cout << SolveMap_BottomRightSmallMap(imap, errplayer, errmark, errgrid, dist, true, R"(D:\output\wtdistmeas\)") << endl;
    printf("ep%lf\nem%lf\neg%lf\ndi%lf\n", errplayer, errmark, errgrid, dist);
    system("pause");
}

void toast_test()
{
    toast("test message 1111",2);
    this_thread::sleep_for(chrono::milliseconds(1000));

    toast("test message 2222",0.5);
    this_thread::sleep_for(chrono::milliseconds(1000));

    toast("test message 3333",2);
    this_thread::sleep_for(chrono::milliseconds(1000));
    while(true)
        this_thread::sleep_for(chrono::milliseconds(1000));

    system("pause");
}

int main(int argc,char**argv)
{
    //solvemapfromdisk();
    //return 0;
    while (true)
    {
        if (isKBDown(VK_F11))
        {
            for (size_t i = 0; i < 3; i++)
            {
                MessageBeep(MB_OK);
                this_thread::sleep_for(chrono::milliseconds(250));
            }
            break;
        }
        //if (isKBDown(192)) //~ key
        if (isKBDown(VK_F12))
        {
            //wait for the yellow mark's settle, if the function key and distmeas key share the same one
            this_thread::sleep_for(chrono::milliseconds(500));
            double errplayer=-1, errmark=-1, errgrid=-1;
            double dist=-1;
            auto iscrrgba = Screenshot().getScreenshot();
            Mat iscrrgb;
            RGBA2RGB(iscrrgba, iscrrgb);
            char buf[256] = { 0 };
            auto ret = SolveMap_BottomRightSmallMap(iscrrgb, errplayer, errgrid, errmark, dist);
            auto scenceautosave = [iscrrgb]()
            {
                double errplayer = -1, errmark = -1, errgrid = -1;
                double dist = -1;
                time_t t = time(NULL);
                struct tm* now_time = localtime(&t);
                char outputpath[256];
                sprintf(outputpath, "%d,%02d,%02d,%02d,%02d,%02d\\",
                    now_time->tm_year + 1900, now_time->tm_mon + 1,
                    now_time->tm_mday, now_time->tm_hour, now_time->tm_min,
                    now_time->tm_sec);
                system((string("mkdir \"") + outputpath+ "\"").c_str());
                SolveMap_BottomRightSmallMap(iscrrgb, errplayer, errgrid, errmark, dist, true, outputpath);
            };

            if ("OK"!=ret)
            {
                scenceautosave();
                sprintf(buf, "err_smbr:\nre%s\nep%f\tem%f\teg%f\n",ret.c_str(), errplayer, errmark, errgrid);
                MessageBeep(MB_ICONERROR);
                toast(buf, 2);
                goto loopcontinue;
            }

            if (errplayer > 100 || errmark > 100 || errgrid > 100)
            {
                scenceautosave();
                sprintf(buf, "err_thresh:\nep%f\tem%f\teg%f", errplayer, errmark, errgrid);
                MessageBeep(MB_ICONEXCLAMATION);
                toast(buf, 2);
                goto loopcontinue;
            }
            
            vector<int> example({140,170,180,200, 225,350 });
            sprintf(buf, "d%lf\n", dist);
            string msg = buf;
            for (const auto& e : example)
            {
                sprintf(buf, "%d:\t%lf\n", e, e * dist);
                msg += buf;
            }
            sprintf(buf, "ep%f\tem%f\teg%f\n", errplayer, errmark, errgrid);
            msg += buf;
            MessageBeep(MB_OK);
            toast(msg,5);
        }
loopcontinue:
        this_thread::sleep_for(chrono::milliseconds(250));
    }

}