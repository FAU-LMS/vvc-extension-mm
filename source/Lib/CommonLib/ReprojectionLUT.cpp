//
// Created by regensky on 11/7/22.
//

#include "ReprojectionLUT.h"

void ReprojectionLUT::fill() {
  ArrayXXTCoordPtr cart2DX = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, 1, Eigen::Dynamic>::LinSpaced(m_width + 1, TCoord(m_minX), TCoord(m_maxX)).replicate(m_height + 1, 1));
  ArrayXXTCoordPtr cart2DY = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, Eigen::Dynamic, 1>::LinSpaced(m_height + 1, TCoord(m_minY), TCoord(m_maxY)).replicate(1, m_width + 1));
  ArrayXXTCoordPtr cart2DXMapped, cart2DYMapped;
  std::tie(cart2DXMapped, cart2DYMapped) = m_func({cart2DX, cart2DY});

  m_lut.resize(m_width * m_height);
  for (int i = 0; i < m_height; ++i) {
    for (int j = 0; j < m_width; ++j) {
//      if(std::abs(cart2DX->coeff(i, j)) < 10 && std::abs(cart2DY->coeff(i, j)) < 10) {
//        std::cout << cart2DX->coeff(i, j) << " " << cart2DY->coeff(i, j) << ": " << cart2DXMapped->coeff(i, j) << ", " << cart2DYMapped->coeff(i, j) << std::endl;
//      }
      m_lut[i * m_width + j] = { cart2DXMapped->coeff(i, j), cart2DYMapped->coeff(i, j) };
    }
  }
}

Array2TCoord ReprojectionLUT::operator()(const Array2TCoord &cart2D) const
{
  // Nearest-neighbor
  int row = int(std::round(cart2D.y())) - m_minY;
  int column = int(std::round(cart2D.x())) - m_minX;

  if (row < 0 || row >= m_height || column < 0  || column >= m_width) {
    return {NAN, NAN};
  }

  return m_lut[row * m_width + column];
}
