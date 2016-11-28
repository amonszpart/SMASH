//
// Created by bontius on 28/04/16.
//

#ifndef TRACKVIDEO_PHYS_CUBOIDIO_HPP
#define TRACKVIDEO_PHYS_CUBOIDIO_HPP

#include "tracking/phys/typedefsGeometry.h"
#include "tracking/phys/typedefs.h" // SHAPE
#include "tracking/common/typedefs.h"
#include "picojson/picojson.h"
#include "picojson/picojsonUtil.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <tracking/common/trackFwDecl.h>
#include <tracking/annot/cuboid.h>


namespace tracking {
  namespace bundle_physics {
    namespace io {
      template <typename _CuboidT>
      void writeCuboids(std::vector<_CuboidT> const&cuboids, std::string const& path, typename _CuboidT::Scalar cSqr) {
          using Vector3 = typename _CuboidT::Vector3;
          std::cout << "writecuboids entered" << std::endl;
          std::ofstream   f(path);
          picojson::array array;
          for (_CuboidT const &cuboid : cuboids) {
              picojson::object object;
              object["name"]  = picojson::value(cuboid.getName());
              object["objId"] = picojson::value(static_cast<double>(cuboid.getObjId()));
              object["mass"]  = picojson::value(cuboid.getMass());
              if (cSqr > 0.)
                  object["c"] = picojson::value(std::sqrt(cSqr));
              else
                  object["c"]   = picojson::value(0.);
              object["size"]    = picojson::vector3ToPicojsonObject(cuboid.getSize());
              object["shape"]   = picojson::value("BOX");
              object["inertia"] = picojson::vector3ToPicojsonObject(Vector3::Zero());

              picojson::object log;
              for (auto const &frameIdAndPoseLoc : cuboid.getStates()) {
                  const FrameId &frameId = frameIdAndPoseLoc.first;

                  picojson::object observation;
                  if (frameIdAndPoseLoc.second.hasPos())
                      observation["pos"]    = picojson::vector3ToPicojsonObject(frameIdAndPoseLoc.second.getPosition());
                  if (frameIdAndPoseLoc.second.hasPose())
                      observation["pose"]   = picojson::vector4ToPicojsonObject(frameIdAndPoseLoc.second.getPose().coeffs());
                  if (frameIdAndPoseLoc.second.hasLinVel())
                      observation["linVel"] = picojson::vector3ToPicojsonObject(frameIdAndPoseLoc.second.getLinVel());
                  if (frameIdAndPoseLoc.second.hasAngVel())
                      observation["angVel"] = picojson::vector3ToPicojsonObject(frameIdAndPoseLoc.second.getAngVel());

                  log[std::to_string(frameId)] = picojson::value(observation);
              }
              object["log"]     = picojson::value(log);
              array.emplace_back(object);
          }
          f << picojson::value(array).serialize();
          f.close();
          std::cout << "wrote to " << path << std::endl;
      } //...writeCuboids

      template <typename _CuboidT>
      int readCuboids(std::vector<_CuboidT> &cuboids, const std::string &inPath) {
          using Vector3 = typename _CuboidT::Vector3;
          // open input text file
          std::ifstream inFile;
          inFile.open(inPath.c_str());
          if (!inFile.is_open()) {
              std::cerr << "[" << __func__ << "]: could not open " << inPath << std::endl;
              return EXIT_FAILURE;
          }

          // parse json object
          picojson::value jsonData;
          std::string     err = picojson::parse(jsonData, inFile);
          if (!err.empty()) {
              std::cerr << "[" << __func__ << "]: could not parse " << err << std::endl;
              return EXIT_FAILURE;
          }


          int entryIndex = 0;
          while (jsonData.contains(entryIndex)) {
              picojson::object object = jsonData.get(entryIndex).template get<picojson::object>();
              _CuboidT         cuboid;
              for (picojson::object::const_iterator entryIt = object.begin(); entryIt != object.end(); ++entryIt) {
                  if (entryIt->first.compare("name") == 0)
                      cuboid.setName(entryIt->second.to_str());
                  else if (entryIt->first.compare("objId") == 0)
                      cuboid.setObjId(std::atol(entryIt->second.to_str().c_str()));
                  else if (entryIt->first.compare("size") == 0)
                      picojson::parseVector3(cuboid.getSize(), entryIt->second.template get<picojson::object>());
                  else if (entryIt->first.compare("mass") == 0)
                      cuboid.setMass(std::atof(entryIt->second.to_str().c_str()));
                  else if (entryIt->first.compare("shape") == 0) {
                      if (entryIt->second.to_str().compare("BOX") == 0)
                          cuboid.setShape(SHAPE::BOX);
                      else if (entryIt->second.to_str().compare("SPHERE") == 0)
                          cuboid.setShape(SHAPE::SPHERE);
                      else if (entryIt->second.to_str().compare("ELLIPSOID") == 0)
                          cuboid.setShape(SHAPE::ELLIPSOID);
                      else if (entryIt->second.to_str().compare("BOX_HOLLOW") == 0)
                          cuboid.setShape(SHAPE::BOX_HOLLOW);
                      else {
                          std::cerr << "wtf, unknown shape: " << entryIt->second.to_str() << std::endl;
                          throw new std::runtime_error("unknown shape");
                      }
                  } else if (entryIt->first.compare("log") == 0) {
                      picojson::object                      log   = entryIt->second.template get<picojson::object>();
                      for (picojson::object::const_iterator logIt = log.begin(); logIt != log.end(); ++logIt) {
                          const FrameId                         frameId  = std::atol(logIt->first.c_str());
                          //std::cout << "frameId: " << frameId << std::endl;
                          picojson::object                      logEntry = logIt->second.template get<picojson::object>();
                          for (picojson::object::const_iterator fieldIt  = logEntry.begin(); fieldIt != logEntry.end(); ++fieldIt) {
                              if (fieldIt->first.compare("pos") == 0) {
                                  Vector3 pos(Vector3::Zero());
                                  picojson::parseVector3(pos, fieldIt->second.template get<picojson::object>());
                                  cuboid.setPosition(frameId, pos);
                              }
                              else if (fieldIt->first.compare("pose") == 0) {
                                  Eigen::Matrix<typename Vector3::Scalar, 4, 1> pose(Eigen::Matrix<typename Vector3::Scalar, 4, 1>::Zero());
                                  picojson::parseVector4(pose, fieldIt->second.template get<picojson::object>());
                                  cuboid.setPose(frameId, QuaternionT(pose(3), pose(0), pose(1), pose(2)));
                              }
                              else if (fieldIt->first.compare("linVel") == 0) {
                                  Vector3 linVel(Vector3::Zero());
                                  picojson::parseVector3(linVel, fieldIt->second.template get<picojson::object>());
                                  cuboid.setLinVel(frameId, linVel);
                              }
                              else if (fieldIt->first.compare("angVel") == 0) {
                                  Vector3 angVel(Vector3::Zero());
                                  picojson::parseVector3(angVel, fieldIt->second.template get<picojson::object>());
                                  cuboid.setAngVel(frameId, angVel);
                              }
                              else
                                  std::cerr << "[" << __func__ << "]: " << "no parsing for key in log: " << fieldIt->first << std::endl;
                          } //...logEntry fields
                      } //...log entries
                  } //...if log
                  else
                      std::cerr << "[" << __func__ << "]: " << "no parsing for key: " << entryIt->first << std::endl;
              } //...entries
              cuboids.push_back(cuboid);
              ++entryIndex;
          } // ...objects

          inFile.close();

          std::sort(cuboids.begin(),cuboids.end(),[](Cuboid const& a, Cuboid const& b){ return a.getObjId() < b.getObjId();});
          return cuboids.size();
      } //...readCuboids

      template <typename _CuboidT>
      int readCuboids(std::vector<_CuboidT> &cuboids, const std::vector<std::string> &inPaths) {
          cuboids.clear();
          std::vector<_CuboidT> tmp;
          for (auto const     &path : inPaths) {
              tmp.clear();
              tracking::bundle_physics::io::readCuboids(tmp, path);
              cuboids.insert(cuboids.end(), tmp.begin(), tmp.end());
          }
          std::sort(cuboids.begin(),cuboids.end(),[](Cuboid const& a, Cuboid const& b){ return a.getObjId() < b.getObjId();});
          return cuboids.size();
      } //...readCuboids
    } //...ns io
  } //...ns bundle_physics
} //...ns tracking
#endif //TRACKVIDEO_PHYS_CUBOIDIO_HPP
