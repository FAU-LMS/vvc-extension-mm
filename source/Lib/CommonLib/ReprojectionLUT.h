//
// Created by regensky on 11/7/22.
//

#pragma once

#include "Coordinate.h"
#include "Unit.h"
#include <functional>

class ReprojectionLUT
{
public:
  ReprojectionLUT(): m_minX(0), m_maxX(0), m_minY(0), m_maxY(0), m_width(0), m_height(0), m_func(nullptr) {}
  ReprojectionLUT(int minX, int maxX, int minY, int maxY, std::function<ArrayXXTCoordPtrPair(ArrayXXTCoordPtrPair)> func):
                  m_minX(minX), m_maxX(maxX), m_minY(minY), m_maxY(maxY), m_func(std::move(func)) {
    m_width = m_maxX - m_minX;
    m_height = m_maxY - m_minY;
  }

  void fill();

  Array2TCoord operator() (const Array2TCoord &cart2D) const;

protected:
  int m_minX;
  int m_maxX;
  int m_minY;
  int m_maxY;
  int m_width;
  int m_height;
  std::function<ArrayXXTCoordPtrPair(ArrayXXTCoordPtrPair)> m_func;

  std::vector<Array2TCoord> m_lut;
};
