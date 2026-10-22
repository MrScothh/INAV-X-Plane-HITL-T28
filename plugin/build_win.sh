#!/usr/bin/env bash
# Builds the plugin for Windows into build/win.xpl with a MinGW-w64 g++ (WinLibs, GCC 13 or newer) on the PATH.
# Installing it: close X-Plane and copy build/win.xpl over <aircraft>/plugins/INAV-X-Plane-HITL/64/win.xpl.
#   bash build_win.sh
set -euo pipefail
cd "$(dirname "$0")/src"
mkdir -p ../build
g++ -std=c++17 -O2 -shared -o ../build/win.xpl *.cpp -ISDK/CHeaders/XPLM -ISDK/CHeaders/Widgets -IOpenAL/include \
    -DIBM=1 -DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DXPLM301=1 -DXPLM302=1 -DXPLM303=1 -DWINVER=0x0601 \
    -D_WIN32_WINNT=0x0601 -D_CRT_SECURE_NO_WARNINGS -LSDK/Libraries/Win -l:XPLM_64.lib -l:XPWidgets_64.lib \
    -LOpenAL/libs/Win64 -lOpenAL32 -lopengl32 -lglu32 -lwinmm -lws2_32 -static-libgcc -static-libstdc++ -static
ls -la ../build/win.xpl
