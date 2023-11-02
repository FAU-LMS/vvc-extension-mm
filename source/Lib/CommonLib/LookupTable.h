//
// Created by regensky on 9/6/21.
//

#pragma once

#include "CommonDef.h"
#include "Coordinate.h"


/// Lookup table for fast function approximation.
class LookupTable {
public:
  LookupTable() = default;
  LookupTable(std::function<TCoord(TCoord)> function, std::pair<TCoord, TCoord> range, int samples);

  ArrayXXTCoordPtr lookup(ArrayXXTCoordPtr value) const;
  TCoord lookup(TCoord value) const;

  ArrayXXTCoordPtr inverseLookup(ArrayXXTCoordPtr value) const;
  TCoord inverseLookup(TCoord value) const;

protected:
  void init(const std::function<TCoord(TCoord)> &function);
  std::pair<int, bool> findInsertIdx(TCoord value) const;

protected:
  ArrayXTCoord m_inputs;
  ArrayXTCoord m_outputs;
  std::pair<TCoord, TCoord> m_range;
  int m_samples;
};
