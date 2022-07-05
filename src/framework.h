#pragma once

#define NOMINMAX // Get rid of windows' annoying min and max definitions

#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN // Exclude rarely-used stuff from Windows headers
    // Windows Header Files
    #include <windows.h>
    #define DllExport extern "C"  __declspec( dllexport )
#endif

#ifdef __unix__
    #define DllExport extern "C"
#endif
