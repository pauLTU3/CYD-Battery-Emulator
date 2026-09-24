/*
 * PROJECT:   LVGL PC Simulator using Visual Studio
 * FILE:      LVGL.Simulator.cpp
 * PURPOSE:   Implementation for LVGL ported to Windows Desktop
 *
 * LICENSE:   The MIT License
 *
 * DEVELOPER: Mouri_Naruto (Mouri_Naruto AT Outlook.com)
 */

#include <Windows.h>

#include "resource.h"

#if _MSC_VER >= 1200
 // Disable compilation warnings.
#pragma warning(push)
// nonstandard extension used : bit field types other than int
#pragma warning(disable:4214)
// 'conversion' conversion from 'type1' to 'type2', possible loss of data
#pragma warning(disable:4244)
#endif

#include "lvgl/lvgl.h"
#include "lv_drivers/win32drv/win32drv.h"
#include "ui/ui.h"

#if _MSC_VER >= 1200
// Restore compilation warnings.
#pragma warning(pop)
#endif

#include <stdio.h>

static BOOL CALLBACK customizeWindowTitleCallback(HWND hWnd, LPARAM lparam);

int APIENTRY WinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPSTR szCmdLine, _In_ int nCmdShow)
{
    lv_init();

    if (!lv_win32_init(
        hInstance,
        SW_SHOWNORMAL,
        WIN32DRV_HOR_RES,
        WIN32DRV_VER_RES,
        LoadIconW(GetModuleHandleW(NULL), MAKEINTRESOURCE(IDI_SQUARELINE))))
    {
        return -1;
    }

    //Uncomment it to create a default group for TAB key based navigation
    //lv_win32_add_all_input_devices_to_group(NULL);

    ui_init();

    // Set a custom title for the newly created window
    EnumWindows(customizeWindowTitleCallback, (LPARAM)(hInstance));

    // ----------------------------------
    // Task handler loop
    // ----------------------------------

    while (!lv_win32_quit_signal)
    {
        /* Periodically call the lv_task handler.
         * It could be done in a timer interrupt or an OS task too.*/
        lv_task_handler();
        Sleep(5);
    }

    return 0;
}

static BOOL CALLBACK customizeWindowTitleCallback(HWND hWnd, LPARAM lparam)
{
    if ((HINSTANCE)GetWindowLongPtrA(hWnd, GWLP_HINSTANCE) == (HINSTANCE)lparam)
    {
        SetWindowTextA(hWnd, "SquareLine Studio Simulator for Windows");
    }

    return TRUE;
}
