
#include "stdafx.h"
#include "toast.h"
const auto BACKGROUNDCOLOR = RGB(0, 0, 0);
const auto FRONTROUNDCOLOR = RGB(255, 255, 255);

string info;
mutex mtex;

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{

	switch (message)
	{
	case WM_LBUTTONDOWN:
		break;
	case WM_LBUTTONUP:
		break;
	case WM_MOUSEMOVE:
		break;
	case WM_PAINT:
		{
			PAINTSTRUCT ps;
			HDC hDC;
			RECT rt;
			hDC = BeginPaint(hWnd, &ps);

			//modify font
			HFONT hFont = CreateFontA(30, 15, 0, 0, FW_BOLD, false, false, false, ANSI_CHARSET, OUT_CHARACTER_PRECIS, CLIP_CHARACTER_PRECIS, ANTIALIASED_QUALITY, VARIABLE_PITCH, "Monaco");
			HGDIOBJ holdfont = SelectObject(hDC, hFont);

			SetBkColor(hDC, BACKGROUNDCOLOR);
			SetTextColor(hDC, FRONTROUNDCOLOR);

			GetClientRect(hWnd, &rt);
			DrawTextA(hDC, info.c_str(), (int)info.size(), &rt, DT_LEFT);
			SelectObject(hDC, holdfont);
			DeleteObject(hFont);
			EndPaint(hWnd, &ps);
		}

		break;
	case WM_TIMER:
	case WM_CLOSE:
		DestroyWindow(hWnd);
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

void windowmainloop(double t)
{
	HWND hWnd;
	MSG msg;
	HINSTANCE hInstance = GetModuleHandle(NULL);


	TCHAR szWindowClass[] = TEXT("WinApp");
	// Register Class 
	{
		WNDCLASSEX wcex;

		wcex.cbSize = sizeof(WNDCLASSEX);

		wcex.style = CS_HREDRAW | CS_VREDRAW | CS_DBLCLKS;
		wcex.lpfnWndProc = (WNDPROC)WndProc;
		wcex.cbClsExtra = 0;
		wcex.cbWndExtra = 0;
		wcex.hInstance = hInstance;
		wcex.hIcon = NULL;
		wcex.hCursor = NULL;
		HBRUSH hBrush = CreateSolidBrush(BACKGROUNDCOLOR);
		wcex.hbrBackground = hBrush;
		wcex.lpszMenuName = NULL;
		wcex.lpszClassName = szWindowClass;
		wcex.hIconSm = NULL;

		RegisterClassEx(&wcex);
	}

	// Perform application initialization:
	{
		hWnd = CreateWindowEx(WS_EX_LAYERED | WS_EX_NOACTIVATE, szWindowClass, TEXT("exp"),
			WS_POPUP,
			500, 500, 1000, 1000, NULL, NULL, hInstance, NULL);

		if (!hWnd)
		{
			return;
		}

		ShowWindow(hWnd, SW_SHOW);
		UpdateWindow(hWnd);

		SetLayeredWindowAttributes(hWnd, BACKGROUNDCOLOR, 0, LWA_COLORKEY);

		SetWindowPos(hWnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_DRAWFRAME | SWP_NOSIZE);
	}
	SetTimer(hWnd, NULL, (UINT)(t * 1000), NULL);
	// Main message loop:
	while (GetMessage(&msg, NULL, 0, 0))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}
}
void toast(string _info,double t)
{
	info = _info;
	thread thReader(windowmainloop,t);
	thReader.detach();
}