//
// Created by regensky on 8/16/22.
//

#include "EpipoleList.h"
#include <iostream>

void EpipoleList::addEpipole(const Array3TCoord &epipole, const int curPOC, const int refPOC, bool makeAvailable) {
  EpipoleEntry entry(FloatingFixedConversion::floatingToFixed(epipole, EPIPOLE_PRECISION_FIXED), makeAvailable);
  m_epipoleMap[{curPOC, refPOC}] = entry;
}

Array3TCoord EpipoleList::findEpipole(int curPOC, int refPOC) const
{
  const auto epipoleFixed = findEpipoleFixed(curPOC, refPOC);
  return FloatingFixedConversion::fixedToFloating(epipoleFixed, EPIPOLE_PRECISION_FIXED);
}

Array3Fixed EpipoleList::findEpipoleFixed(int curPOC, int refPOC) const
{
  // Check if explicit entry for given (curPOC, refPOC) combination exists
  auto iter = m_epipoleMap.find({curPOC, refPOC});
  if (iter != m_epipoleMap.end() && iter->second.isAvailable) {
    return iter->second.epipole;
  }

  // Check if per POC entry for given curPOC exist
  iter = m_epipoleMap.find({curPOC, -1});
  if (iter != m_epipoleMap.end() && iter->second.isAvailable) {
    return iter->second.epipole;
  }

  // Check if global entry exists
  iter = m_epipoleMap.find({-1, -1});
  if (iter != m_epipoleMap.end() && iter->second.isAvailable) {
    return iter->second.epipole;
  }

  CHECK(true, "No epipole for given (curPOC, refPOC) combination (" + std::to_string(curPOC) + ", " + std::to_string(refPOC) + ") found.");
}

Array3TCoord EpipoleList::derivePredictor(int curPOC) const
{
  const auto globalEpipoleEntry = m_epipoleMap.at({-1, -1});
  CHECK(!globalEpipoleEntry.isAvailable, "Global epipole is not available.");

  int minPOCDistances[2] = { std::numeric_limits<int>::max(), std::numeric_limits<int>::max() };
  Array3Fixed predictors[2] = { globalEpipoleEntry.epipole, globalEpipoleEntry.epipole };
  for (const auto &item : m_epipoleMap)
  {
    if (!item.second.isAvailable)
    {
      continue;
    }
    int distance = abs(curPOC - item.first.first);
    if (distance < minPOCDistances[0])
    {
      minPOCDistances[0] = distance;
      predictors[0] = item.second.epipole;
    }
    else if (distance < minPOCDistances[1])
    {
      minPOCDistances[1] = distance;
      predictors[1] = item.second.epipole;
    }
  }

  Array3Fixed predictor;
  if (minPOCDistances[0] == minPOCDistances[1])
  {
    predictor = predictors[0] + predictors[1] / 2;
  }
  else
  {
    CHECK(minPOCDistances[0] > minPOCDistances[1], "Smallest POC distance is larger than second smallest POC distance.");
    predictor = predictors[0];
  }

  return FloatingFixedConversion::fixedToFloating(predictor, EPIPOLE_PRECISION_FIXED);
}

bool EpipoleList::hasEpipole(int curPOC, int refPOC) const
{
  const auto iter = m_epipoleMap.find({curPOC, refPOC});
  if (iter != m_epipoleMap.end() && iter->second.isAvailable) {
    return true;
  }
  return false;
}

void EpipoleList::makeAvailable(int curPOC)
{
  for (auto& iter : m_epipoleMap)
  {
    if (iter.first.first == curPOC) {
      iter.second.isAvailable = true;
    }
  }
}

void EpipoleList::printSummary() const
{
  std::cout << "\n\n----- Epipole config -----\n";
  for (auto &iter: m_epipoleMap) {
    if (iter.first.first == -1 && iter.first.second == -1 && iter.second.epipole.isZero()) {
      continue;
    }
    std::cout << iter.first.first << ", " << iter.first.second << ": (" << FloatingFixedConversion::fixedToFloating(iter.second.epipole, EPIPOLE_PRECISION_FIXED) << ")\n";
  }
  std::cout << "----- Epipole config -----\n";
}

