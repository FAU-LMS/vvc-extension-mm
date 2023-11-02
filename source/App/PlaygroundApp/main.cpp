//
// Created by regensky on 02.03.21.
//

#include <iostream>
#include <chrono>
#include <random>
#include <bitset>
#include "Coordinate.h"
#include "Projection.h"
#include "MVReprojection.h"
#include "MVReprojectionLegacy.h"
#include "MotionModels/models.h"
#include "Lib360/TGeometry.h"
#ifdef OpenCV_FOUND
#include "opencv2/opencv.hpp"
#endif
using namespace std;

ArrayXXTCoordPtrPair toPtrPair(Array2TCoord array2TCoord) {
  ArrayXXTCoord coord0(Eigen::Index(1), Eigen::Index(1));
  ArrayXXTCoord coord1(Eigen::Index(1), Eigen::Index(1));
  coord0.coeffRef(0) = array2TCoord.coeff(0);
  coord1.coeffRef(0) = array2TCoord.coeff(1);
  ArrayXXTCoordPtr coord0Ptr = std::make_shared<ArrayXXTCoord>(coord0);
  ArrayXXTCoordPtr coord1Ptr = std::make_shared<ArrayXXTCoord>(coord1);
  return {coord0Ptr, coord1Ptr};
}

ArrayXXTCoordPtr toPtr(TCoord tCoord) {
  ArrayXXTCoord coord(Eigen::Index(1), Eigen::Index(1));
  coord.coeffRef(0) = tCoord;
  ArrayXXTCoordPtr coordPtr = std::make_shared<ArrayXXTCoord>(coord);
  return coordPtr;
}

void printArray(Array2TCoord array) {
  std::cout << "(" << array.x() << ", " << array.y() << ")\n";
}

void printArray(Array3TCoord array) {
  std::cout << "(" << array.x() << ", " << array.y() << ", " << array.z() << ")\n";
}

void printArrayPtr(ArrayXXTCoordPtrPair array) {
  std::cout << "(" << std::get<0>(array)->coeff(0) << ", " << std::get<1>(array)->coeff(0) << ")\n";
}

void printArrayPtr(ArrayXXTCoordPtrTriple array) {
  std::cout << "(" << std::get<0>(array)->coeff(0) << ", " << std::get<1>(array)->coeff(0) << ", " << std::get<2>(array)->coeff(0) << ")\n";
}

bool compare(Array2TCoord array, ArrayXXTCoordPtrPair arrayPtr) {
  TCoord diffX = array.x() - std::get<0>(arrayPtr)->coeff(0);
  TCoord diffY = array.y() - std::get<1>(arrayPtr)->coeff(0);

  return (abs(diffX) < 1 && abs(diffY) < 1);
}

bool compare(Array3TCoord array, ArrayXXTCoordPtrTriple arrayPtr) {
  TCoord diffX = array.x() - std::get<0>(arrayPtr)->coeff(0);
  TCoord diffY = array.y() - std::get<1>(arrayPtr)->coeff(0);
  TCoord diffZ = array.z() - std::get<2>(arrayPtr)->coeff(0);

  return (abs(diffX) < 1 && abs(diffY) < 1 && abs(diffZ) < 1);
}

std::random_device randomDevice;
std::mt19937 generator(randomDevice());
std::uniform_real_distribution<TCoord> distribution(-300, 300);

Array2TCoord randCoord() {
  return {distribution(generator), distribution(generator)};
}

void checkMatch(Array2TCoord array, ArrayXXTCoordPtrPair arrayPtr) {
  if (!compare(array, arrayPtr))
  {
    //printArray2TCoord(cart2DOrig);
    printArray(array);
    printArrayPtr(arrayPtr);
    std::cout << "\n";
  }
}

void checkMatch(Array3TCoord array, ArrayXXTCoordPtrTriple arrayPtr) {
  if (!compare(array, arrayPtr))
  {
    //printArray2TCoord(cart2DOrig);
    printArray(array);
    printArrayPtr(arrayPtr);
    std::cout << "\n";
  }
}

void testMotionModel() {
  Size m_resolution(2216, 1108);
  TCoord m_offset4x4 = 1;
  ArrayXXTCoordPtr cart2DProjX = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, 1, Eigen::Dynamic>::LinSpaced(m_resolution.width / 4, m_offset4x4, TCoord(m_resolution.width - 4) + m_offset4x4).replicate(m_resolution.height / 4, 1));
  ArrayXXTCoordPtr cart2DProjY  = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, Eigen::Dynamic, 1>::LinSpaced(m_resolution.height / 4, m_offset4x4, TCoord(m_resolution.height - 4) + m_offset4x4).replicate(1, m_resolution.width / 4));

  auto* projection = new EquirectangularProjection(m_resolution);
  GeodesicMotionModel motionModel(projection, M_PI_2/1000, GeodesicMotionModel::VISHWANATH_ORIGINAL);
  motionModel.setEpipole({-1, 0, 0});
  motionModel.fillCache({ cart2DProjX, cart2DProjY });

  Position position(0, 0);
  Size subblockSize(4, 4);
  Size size(64, 64);
  Array2TCoord mv = {1, 1};

  auto cart2DProjXBlock = std::make_shared<ArrayXXTCoord>(cart2DProjX->block(position.y/subblockSize.height, position.x/subblockSize.width, size.height/subblockSize.height, size.width/subblockSize.width));
  auto cart2DProjYBlock = std::make_shared<ArrayXXTCoord>(cart2DProjY->block(position.y/subblockSize.height, position.x/subblockSize.width, size.height/subblockSize.height, size.width/subblockSize.width));
  const Array2TCoord blockCenter = {cart2DProjXBlock->mean(), cart2DProjYBlock->mean()};

  ArrayXXTCoordPtrPair cart2DProjMoved = motionModel.modelMotion({cart2DProjXBlock, cart2DProjYBlock}, mv, blockCenter);
  ArrayXXTCoordPtrPair cart2DProjMovedCached = motionModel.modelMotionCached(Position(position.x/subblockSize.width, position.y/subblockSize.height),
                                                                             Size(size.width/subblockSize.width, size.height/subblockSize.height),
                                                                             mv, blockCenter);

  std::cout << std::get<0>(cart2DProjMoved)->isApprox(*std::get<0>(cart2DProjMovedCached)) << std::endl;
  std::cout << std::get<1>(cart2DProjMoved)->isApprox(*std::get<1>(cart2DProjMovedCached)) << std::endl;

  std::cout << (*std::get<1>(cart2DProjMoved) - *std::get<1>(cart2DProjMovedCached)).abs().maxCoeff() << std::endl << std::endl;
//  std::cout << *std::get<1>(cart2DProjMovedCached) << std::endl;
}

void testGEDEpipoleMVDerivation() {
  Size m_resolution(2216, 1108);
  TCoord m_offset4x4 = 1;
  ArrayXXTCoordPtr cart2DProjX = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, 1, Eigen::Dynamic>::LinSpaced(m_resolution.width / 4, m_offset4x4, TCoord(m_resolution.width - 4) + m_offset4x4).replicate(m_resolution.height / 4, 1));
  ArrayXXTCoordPtr cart2DProjY  = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, Eigen::Dynamic, 1>::LinSpaced(m_resolution.height / 4, m_offset4x4, TCoord(m_resolution.height - 4) + m_offset4x4).replicate(1, m_resolution.width / 4));
  SPS sps;
  sps.setUseGED(true);
  sps.setMMOffset4x4(1);
  sps.setUseMMMVP(true);

  EpipoleList epipoleList;
  epipoleList.addEpipole({-0.384, 0.133, 0.742}, 2, -1, true);
  epipoleList.addEpipole({0.872, 0.038, 0.111}, 3, -1, true);

  auto* projection = new EquirectangularProjection(m_resolution);

  MVReprojection mvReprojection;
  mvReprojection.init(projection, m_resolution, &sps, &epipoleList);

  Position position(338, 203);
  Mv mvOrig(19, 11);
  mvOrig.changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
  MotionModelID modelIDOrig = GEODESIC_CAMPOSE;
  MotionModelID modelIDDesired = GEODESIC_CAMPOSE;
  int shiftHor = MV_FRACTIONAL_BITS_INTERNAL;
  int shiftVer = MV_FRACTIONAL_BITS_INTERNAL;
  int curPOCOrig = 2;
  int refPOCOrig = -1;
  int curPOCDesired = 3;
  int refPOCDesired = -1;
  Mv mvDesired = mvReprojection.motionVectorInDesiredMotionModel(position, mvOrig, modelIDOrig, modelIDDesired, shiftHor, shiftVer, curPOCOrig, refPOCOrig, curPOCDesired, refPOCDesired,
                                                                                 position, Size(1, 1), position, Size(1, 1));

  const TCoord mvOrigX = TCoord(mvOrig.hor >> shiftHor) + TCoord(mvOrig.hor & ((1 << shiftHor) - 1))/TCoord(1 << shiftHor);
  const TCoord mvOrigY = TCoord(mvOrig.ver >> shiftVer) + TCoord(mvOrig.ver & ((1 << shiftVer) - 1))/TCoord(1 << shiftVer);
  const Array2TCoord mvOrig_(mvOrigX, mvOrigY);
  const TCoord mvDesiredX = TCoord(mvDesired.hor >> shiftHor) + TCoord(mvDesired.hor & ((1 << shiftHor) - 1))/TCoord(1 << shiftHor);
  const TCoord mvDesiredY = TCoord(mvDesired.ver >> shiftVer) + TCoord(mvDesired.ver & ((1 << shiftVer) - 1))/TCoord(1 << shiftVer);
  const Array2TCoord mvDesired_(mvDesiredX, mvDesiredY);

  std::cout << mvOrig_ << " -> " << mvDesired_ << std::endl;

  GeodesicMotionModel* motionModel = static_cast<GeodesicMotionModel*>(mvReprojection.getMotionModel(GEODESIC_CAMPOSE));

  ArrayXXTCoordPtr cart2DX = std::make_shared<ArrayXXTCoord>(Eigen::Index(1), Eigen::Index(1));
  ArrayXXTCoordPtr cart2DY = std::make_shared<ArrayXXTCoord>(Eigen::Index(1), Eigen::Index(1));
  cart2DX->coeffRef(0) = TCoord(position.x);
  cart2DY->coeffRef(0) = TCoord(position.y);

  motionModel->setEpipole(epipoleList.findEpipole(curPOCOrig, refPOCOrig));
  ArrayXXTCoordPtrPair cart2DShiftOrig = motionModel->modelMotion({cart2DX, cart2DY}, mvOrig_, {position.x, position.y});
  Array2TCoord shiftOrig(std::get<0>(cart2DShiftOrig)->coeff(0), std::get<1>(cart2DShiftOrig)->coeff(0));

  motionModel->setEpipole(epipoleList.findEpipole(curPOCDesired, refPOCDesired));
  ArrayXXTCoordPtrPair cart2DShiftDesired = motionModel->modelMotion({cart2DX, cart2DY}, mvDesired_, {0, 0});
  Array2TCoord shiftDesired(std::get<0>(cart2DShiftDesired)->coeff(0), std::get<1>(cart2DShiftDesired)->coeff(0));

  std::cout << shiftOrig << " -> " << shiftDesired << std::endl;
}

void testMVReprojection() {
  Size m_resolution(2216, 1108);
  TCoord m_offset4x4 = 1;
  ArrayXXTCoordPtr cart2DProjX = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, 1, Eigen::Dynamic>::LinSpaced(m_resolution.width / 4, m_offset4x4, TCoord(m_resolution.width - 4) + m_offset4x4).replicate(m_resolution.height / 4, 1));
  ArrayXXTCoordPtr cart2DProjY  = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, Eigen::Dynamic, 1>::LinSpaced(m_resolution.height / 4, m_offset4x4, TCoord(m_resolution.height - 4) + m_offset4x4).replicate(1, m_resolution.width / 4));
  SPS sps;
  sps.setUseMPA(true);
  sps.setMMOffset4x4(1);
  sps.setUseMMMVP(true);

  EpipoleList epipoleList;

  auto* projection = new EquirectangularProjection(m_resolution);

  MVReprojection mvReprojection;
  mvReprojection.init(projection, m_resolution, &sps, &epipoleList);

  MVReprojectionLegacy mvReprojectionLegacy;
  mvReprojectionLegacy.init(projection, m_resolution, 1);

  Position position(128, 256);
  Size subblockSize(4, 4);
  Size size(64, 64);
  Mv mv(0, 1);
  mv.changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);

  for (int i = 0; i < m_resolution.width - size.width; i+=size.width) {
    for (int j = 0; j < m_resolution.height - size.height; j+=size.height) {
      std::cout << i << ", " << j << std::endl;
      for (int motionModelID = 1; motionModelID < 4; ++motionModelID) {
        for (int mvX = 0; mvX < 32; ++mvX) {
          for (int mvY = 0; mvY < 32; ++mvY) {
            ArrayXXFixedPtrPair cart2DMoved = mvReprojection.reprojectMotionVectorSubblocks({i, j}, size, {mvX, mvY}, MotionModelID(motionModelID), COMPONENT_Y, CHROMA_400, -1, -1);
            ArrayXXFixedPtrPair cart2DMovedLegacy = mvReprojectionLegacy.reprojectMotionVector4x4({i, j}, size, {mvX, mvY}, Viewport(motionModelID), MV_FRACTIONAL_BITS_INTERNAL, MV_FRACTIONAL_BITS_INTERNAL);

            if(!std::get<0>(cart2DMoved)->isApprox(*std::get<0>(cart2DMovedLegacy)) ||
                !std::get<1>(cart2DMoved)->isApprox(*std::get<1>(cart2DMovedLegacy))) {
              std::cout << i << ", " << j << ", " << motionModelID << ", " << mvX << ", " << mvY << std::endl;
            }
          }
        }
      }
    }
  }

  for (int i = 0; i < m_resolution.height; ++i) {
    for (int j = 0; j < m_resolution.width; ++j) {
      position = Position(i, j);
      Mv mvEq = mvReprojection.motionVectorInDesiredMotionModel(position, mv, MPA_LEFT_RIGHT, MPA_TOP_BOTTOM, MV_FRACTIONAL_BITS_INTERNAL, MV_FRACTIONAL_BITS_INTERNAL, -1, -1, -1, -1, position, {}, position, {});
      Mv mvEqLegacy = mvReprojectionLegacy.motionVectorInDesiredViewport(position, mv, LEFT_RIGHT, TOP_BOTTOM, MV_FRACTIONAL_BITS_INTERNAL, MV_FRACTIONAL_BITS_INTERNAL);

      const TCoord mvEqX = TCoord(mvEq.hor >> MV_FRACTIONAL_BITS_INTERNAL) + TCoord(mvEq.hor & ((1 << MV_FRACTIONAL_BITS_INTERNAL) - 1))/TCoord(1 << MV_FRACTIONAL_BITS_INTERNAL);
      const TCoord mvEqY = TCoord(mvEq.ver >> MV_FRACTIONAL_BITS_INTERNAL) + TCoord(mvEq.ver & ((1 << MV_FRACTIONAL_BITS_INTERNAL) - 1))/TCoord(1 << MV_FRACTIONAL_BITS_INTERNAL);

      const TCoord mvEqLegacyX = TCoord(mvEqLegacy.hor >> MV_FRACTIONAL_BITS_INTERNAL) + TCoord(mvEqLegacy.hor & ((1 << MV_FRACTIONAL_BITS_INTERNAL) - 1))/TCoord(1 << MV_FRACTIONAL_BITS_INTERNAL);
      const TCoord mvEqLegacyY = TCoord(mvEqLegacy.ver >> MV_FRACTIONAL_BITS_INTERNAL) + TCoord(mvEqLegacy.ver & ((1 << MV_FRACTIONAL_BITS_INTERNAL) - 1))/TCoord(1 << MV_FRACTIONAL_BITS_INTERNAL);

      if (mvEqX != mvEqLegacyX or mvEqY != mvEqLegacyY) {
        std::cout << mvEqX << ", " << mvEqY << std::endl;
        std::cout << mvEqLegacyX << ", " << mvEqLegacyY << std::endl;
      }
    }
  }
}

void testMMMVP() {
  Size resolution(2216, 1108);
  TCoord offset4x4 = 1;
  auto* projection = new EquirectangularProjection(resolution);
  RotationalMotionModel motionModel(projection, M_PI_2/resolution.height);
//  motionModel.setEpipole({-1, 0, 0});

  GeodesicMotionModel motionModel2(projection, M_PI_2/resolution.height, GeodesicMotionModel::REGENSKY_GEO_GLOBAL);
  motionModel2.setEpipole({-1, 0, 0});

  Array2TCoord anchorPosition(256, 128);
  Array2TCoord candidateBlockCenter = anchorPosition - Array2TCoord(64, 64);
  Array2TCoord currentBlockCenter = anchorPosition + Array2TCoord(-64, 64);
  Array2TCoord motionVector(96, 0);

  ArrayXXTCoordPtr cart2DX = std::make_shared<ArrayXXTCoord>(Eigen::Index(1), Eigen::Index(1));
  ArrayXXTCoordPtr cart2DY = std::make_shared<ArrayXXTCoord>(Eigen::Index(1), Eigen::Index(1));
  cart2DX->coeffRef(0) = anchorPosition.x();
  cart2DY->coeffRef(0) = anchorPosition.y();
  const auto shiftedBlockPosition = motionModel2.modelMotion({cart2DX, cart2DY}, motionVector, candidateBlockCenter);
  const auto shiftedBlockPositionX = std::get<0>(shiftedBlockPosition)->coeff(0);
  const auto shiftedBlockPositionY = std::get<1>(shiftedBlockPosition)->coeff(0);
  const auto equivalentMotionVector = motionModel.motionVectorForEquivalentPixelShiftAt(Position(anchorPosition.x(), anchorPosition.y()),
                                                                                        {shiftedBlockPositionX, shiftedBlockPositionY},
                                                                                        currentBlockCenter);

  const auto shiftedBlockPositionNew = motionModel.modelMotion({cart2DX, cart2DY}, equivalentMotionVector, currentBlockCenter);
  const auto shiftedBlockPositionNewX = std::get<0>(shiftedBlockPositionNew)->coeff(0);
  const auto shiftedBlockPositionNewY = std::get<1>(shiftedBlockPositionNew)->coeff(0);

  std::cout << motionVector << " --> " << equivalentMotionVector << std::endl;
  std::cout << shiftedBlockPositionX << ", " << shiftedBlockPositionY << " --> " << shiftedBlockPositionNewX << ", " << shiftedBlockPositionNewY << std::endl;
}

#ifdef OpenCV_FOUND
class MotionModelVisualizer {
public:
  MotionModelVisualizer() {
    m_width = 3072;
    m_height = 1536;

    m_cart2DX = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, 1, Eigen::Dynamic>::LinSpaced(m_width, TCoord(0), TCoord(m_width - 1)).replicate(m_height, 1));
    m_cart2DY = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, Eigen::Dynamic, 1>::LinSpaced(m_height, TCoord(0), TCoord(m_height - 1)).replicate(1, m_width));

    std::ifstream infile("/CLUSTERHOMES/regensky/Resources/360/yuv400/Balboa_3072x1536_60fps_8bit_400_erp.yuv", ios::binary | ios::in);
    auto* buffer = new uint8_t[m_width * m_height];
    infile.read((char*)buffer, m_width * m_height);
    infile.close();
    m_mat = cv::Mat(m_height, m_width, CV_8UC1, buffer);
  }

  void makeVideoAll() {
    const auto projection = new EquirectangularProjection(Size(m_width, m_height), 0);
    std::map<std::string, MotionModel*> motionModels;
//    motionModels["TRANS"] = new TranslationalMotionModel();
//    motionModels["MPA_FRONTBACK"] = new MotionPlaneAdaptiveMotionModel(projection, MPA_FRONT_BACK);
//    motionModels["MPA_LEFTRIGHT"] = new MotionPlaneAdaptiveMotionModel(projection, MPA_LEFT_RIGHT);
//    motionModels["MPA_TOPBOTTOM"] = new MotionPlaneAdaptiveMotionModel(projection, MPA_TOP_BOTTOM);
    motionModels["3DT"] = new ThreeDTranslationalMotionModel(projection);
    motionModels["TAN"] = new TangentialMotionModel(projection, M_PI/m_height);
    motionModels["ROT"] = new RotationalMotionModel(projection, M_PI/m_height);
    motionModels["GED_X"] = new GeodesicMotionModel(projection, M_PI/m_height, GeodesicMotionModel::VISHWANATH_ORIGINAL);
    static_cast<GeodesicMotionModel*>(motionModels["GED_X"])->setEpipole({1, 0, 0});
    motionModels["GED_Y"] = new GeodesicMotionModel(projection, M_PI/m_height, GeodesicMotionModel::VISHWANATH_ORIGINAL);
    static_cast<GeodesicMotionModel*>(motionModels["GED_Y"])->setEpipole({0, 1, 0});
    motionModels["GED_Z"] = new GeodesicMotionModel(projection, M_PI/m_height, GeodesicMotionModel::VISHWANATH_ORIGINAL);
    static_cast<GeodesicMotionModel*>(motionModels["GED_Z"])->setEpipole({0, 0, 1});
    motionModels["GED_X_MOD"] = new GeodesicMotionModel(projection, M_PI/m_height, GeodesicMotionModel::VISHWANATH_MODULATED);
    static_cast<GeodesicMotionModel*>(motionModels["GED_X_MOD"])->setEpipole({1, 0, 0});
    motionModels["GED_Y_MOD"] = new GeodesicMotionModel(projection, M_PI/m_height, GeodesicMotionModel::VISHWANATH_MODULATED);
    static_cast<GeodesicMotionModel*>(motionModels["GED_Y_MOD"])->setEpipole({0, 1, 0});
    motionModels["GED_Z_MOD"] = new GeodesicMotionModel(projection, M_PI/m_height, GeodesicMotionModel::VISHWANATH_MODULATED);
    static_cast<GeodesicMotionModel*>(motionModels["GED_Z_MOD"])->setEpipole({0, 0, 1});
    motionModels["GED_X_GEO_G"] = new GeodesicMotionModel(projection, M_PI/m_height, GeodesicMotionModel::REGENSKY_GEO_GLOBAL);
    static_cast<GeodesicMotionModel*>(motionModels["GED_X_GEO_G"])->setEpipole({1, 0, 0});
    motionModels["GED_Y_GEO_G"] = new GeodesicMotionModel(projection, M_PI/m_height, GeodesicMotionModel::REGENSKY_GEO_GLOBAL);
    static_cast<GeodesicMotionModel*>(motionModels["GED_Y_GEO_G"])->setEpipole({0, 1, 0});
    motionModels["GED_Z_GEO_G"] = new GeodesicMotionModel(projection, M_PI/m_height, GeodesicMotionModel::REGENSKY_GEO_GLOBAL);
    static_cast<GeodesicMotionModel*>(motionModels["GED_Z_GEO_G"])->setEpipole({0, 0, 1});
    motionModels["GED_X_GEO_B"] = new GeodesicMotionModel(projection, M_PI/m_height, GeodesicMotionModel::REGENSKY_GEO_BLOCK);
    static_cast<GeodesicMotionModel*>(motionModels["GED_X_GEO_B"])->setEpipole({1, 0, 0});
    motionModels["GED_Y_GEO_B"] = new GeodesicMotionModel(projection, M_PI/m_height, GeodesicMotionModel::REGENSKY_GEO_BLOCK);
    static_cast<GeodesicMotionModel*>(motionModels["GED_Y_GEO_B"])->setEpipole({0, 1, 0});
    motionModels["GED_Z_GEO_B"] = new GeodesicMotionModel(projection, M_PI/m_height, GeodesicMotionModel::REGENSKY_GEO_BLOCK);
    static_cast<GeodesicMotionModel*>(motionModels["GED_Z_GEO_B"])->setEpipole({0, 0, 1});

    for (auto const& entry : motionModels) {
      std::cout << std::string("/home/regensky/Development/vtm/output/") + entry.first + std::string(".avi") << std::endl;
      cv::VideoWriter videoWriter(std::string("/home/regensky/Development/vtm/output/") + entry.first + std::string(".avi"), cv::VideoWriter::fourcc('M','J','P','G'), double(30), cv::Size(m_width, m_height));
      makeVideo(videoWriter, entry.second, entry.first);
      videoWriter.release();
    }
  }

  void makeVideo(cv::VideoWriter &videoWriter, MotionModel* motionModel, const std::string &name) {
    double mvX = 0;
    double mvY = 0;
    for (int i = 0; i < 30; i++) {
      mvX = (double(i)/29.) * 96;
      videoWriter.write(makeFrame(mvX, mvY, motionModel, name));
    }

    for (int j = 0; j < 30; j++) {
      mvY = (double(j)/29.) * 96;
      videoWriter.write(makeFrame(mvX, mvY, motionModel, name));
    }

    for (int i = 29; i > -30; i--) {
      mvX = (double(i)/29.) * 96;
      videoWriter.write(makeFrame(mvX, mvY, motionModel, name));
    }

    for (int j = 29; j > -30; j--) {
      mvY = (double(j)/29.) * 96;
      videoWriter.write(makeFrame(mvX, mvY, motionModel, name));
    }

    for (int i = -29; i < 30; i++) {
      mvX = (double(i)/29.) * 96;
      videoWriter.write(makeFrame(mvX, mvY, motionModel, name));
    }

    for (int j = -29; j <= 0; j++) {
      mvY = (double(j)/29.) * 96;
      videoWriter.write(makeFrame(mvX, mvY, motionModel, name));
    }

    for (int i = 29; i >= 0; i--) {
      mvX = (double(i)/29.) * 96;
      videoWriter.write(makeFrame(mvX, mvY, motionModel, name));
    }
    std::cout << "Done" << std::endl;
  }

  cv::Mat makeFrame(double mvX, double mvY, MotionModel* motionModel, const std::string &name) {
    cv::Mat matNew = cv::Mat(m_height, m_width, CV_8UC1);
    int blocksize = 128;
    int subblocksize = 4;
    for (int i_block=0; i_block < m_height; i_block += blocksize)
    {
      for (int j_block = 0; j_block < m_width; j_block += blocksize)
      {
        ArrayXXTCoordPtrPair cart2D = {
          std::make_shared<ArrayXXTCoord>(m_cart2DX->block(i_block, j_block, blocksize, blocksize)),
          std::make_shared<ArrayXXTCoord>(m_cart2DY->block(i_block, j_block, blocksize, blocksize))
        };
        Array2TCoord blockCenter = Array2TCoord(j_block, i_block) + (Array2TCoord(blocksize, blocksize) - 1) / TCoord(2);

        ArrayXXTCoordPtrPair cart2DMoved  = motionModel->modelMotion(cart2D, { mvX, mvY }, blockCenter);
        auto                 cart2DMovedX = *std::get<0>(cart2DMoved);
        auto                 cart2DMovedY = *std::get<1>(cart2DMoved);

        auto xPos = cart2DMovedX.round().cast<int>().min(m_width - 1).max(0);
        auto yPos = cart2DMovedY.round().cast<int>().min(m_height - 1).max(0);

        for (int i = i_block; i < i_block + blocksize; i += subblocksize)
        {
          for (int j = j_block; j < j_block + blocksize; j += subblocksize)
          {
            int mvXx = xPos(i - i_block + 1, j - j_block + 1) - m_cart2DX->coeff(i + 1, j + 1);
            int mvYy = yPos(i - i_block + 1, j - j_block + 1) - m_cart2DY->coeff(i + 1, j + 1);
            for (int m = i; m < i + subblocksize; ++m)
            {
              for (int n = j; n < j + subblocksize; ++n)
              {
                auto exX = std::min(std::max(int(m_cart2DX->coeff(m, n)) + mvXx, 0), m_width - 1);
                auto exY = std::min(std::max(int(m_cart2DY->coeff(m, n)) + mvYy, 0), m_height - 1);
                matNew.at<unsigned char>(m, n) = m_mat.at<unsigned char>(exY, exX);
              }
            }
            //          matNew.at<unsigned char>(i, j) = mat.at<unsigned char>(yPos(i - i_block, j - j_block),
            //                                                                 xPos(i - i_block, j - j_block));
          }
        }
      }
    }
    cv::Mat frameRGB;
    cv::cvtColor(matNew, frameRGB, cv::COLOR_GRAY2BGR);
    cv::putText(frameRGB, "MVx: " + std::to_string(mvX).substr(0, std::to_string(mvX).find('.')), cv::Point(10, 30), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 255, 0), 2);
    cv::putText(frameRGB, "MVy: " + std::to_string(mvY).substr(0, std::to_string(mvY).find('.')), cv::Point(10, 90), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 255, 0), 2);
    cv::putText(frameRGB, name, cv::Point(m_width/2, 30), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 255, 0), 2);
    return frameRGB;
  }

  int m_width;
  int m_height;
  cv::Mat m_mat;
  ArrayXXTCoordPtr m_cart2DX;
  ArrayXXTCoordPtr m_cart2DY;
};
#endif

int main(int argc, char* argv[]) {
#ifdef OpenCV_FOUND
    testMMMVP();
//    MotionModelVisualizer vis;
//    vis.makeVideoAll();
#else
  testGEDEpipoleMVDerivation();
  return 0;
#endif
}
