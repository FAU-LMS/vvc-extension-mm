//
// Created by regensky on 9/6/21.
//

#include "LookupTable.h"

LookupTable::LookupTable(std::function<TCoord(TCoord)> function, std::pair<TCoord, TCoord> range, int samples)
{
  m_range = range;
  m_samples = samples;
  init(function);
}

void LookupTable::init(const std::function<TCoord(TCoord)> &function)
{
  m_inputs = ArrayXTCoord::LinSpaced(m_samples, m_range.first, m_range.second);
  m_outputs = m_inputs.unaryExpr(function);
}

TCoord LookupTable::lookup(TCoord value) const
{
  return m_outputs(static_cast<int>(std::round(std::max(TCoord(0), std::min(TCoord(1), (value - m_range.first) / (m_range.second - m_range.first))) * TCoord(m_samples - 1))));
}

ArrayXXTCoordPtr LookupTable::lookup(ArrayXXTCoordPtr value) const
{
  Eigen::ArrayXXi indices = (((*value - m_range.first) / (m_range.second - m_range.first)) * TCoord(m_samples - 1)).round().cast<int>().cwiseMax(0).cwiseMin(m_samples-1);
  return std::make_shared<ArrayXXTCoord>(indices.unaryExpr([this](int index){return m_outputs(static_cast<int>(index));}));
}

TCoord LookupTable::inverseLookup(TCoord value) const
{
    std::pair<int, bool> res = findInsertIdx(value);
    if (res.second) {
      return m_inputs[res.first];
    } else {
      if (res.first == 0) {
        return m_inputs[0];
      } else if (res.first == m_outputs.size()) {
        return m_inputs[res.first - 1];
      } else {
        TCoord dLow = value - m_outputs[res.first - 1];
        TCoord dHigh = m_outputs[res.first] - value;
        if (dLow < dHigh) {
          return m_inputs[res.first - 1];
        } else {
          return m_inputs[res.first];
        }
      }
    }
}

ArrayXXTCoordPtr LookupTable::inverseLookup(ArrayXXTCoordPtr value) const
{
  return std::make_shared<ArrayXXTCoord>(value->unaryExpr([this](TCoord value){return this->inverseLookup(value);}));
}

std::pair<int, bool> LookupTable::findInsertIdx(TCoord value) const
{
  // Find insert index with binary sort.
  int iLow = 0;
  int iHigh = static_cast<int>(m_outputs.size()) - 1;

  while (iLow <= iHigh) {
    int iSplit = (iLow + iHigh) / 2;
    if (m_outputs[iSplit] == value) {
      return {iSplit, true};
    } else if (m_outputs[iSplit] < value) {
      iLow = iSplit + 1;
    } else {
      iHigh = iSplit - 1;
    }
  }
  return {iHigh + 1, false};
}
