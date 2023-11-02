//
// Created by Office on 10.02.21.
//

#ifndef VTM_DEBUG_TOOLS_H
#define VTM_DEBUG_TOOLS_H

#include "Unit.h"

namespace debug_tools {
  void showBuf(const Pel * buf, int stride, int width, int height, int bitdepth, const char* windowID, int delay = 50, const char* windowTitle = nullptr);
  void showYUV(const PelUnitBuf & yuv, int bitdepth, const char* windowID, int delay = 50, const char* windowTitle = nullptr);
}


#endif   // VTM_DEBUG_TOOLS_H
