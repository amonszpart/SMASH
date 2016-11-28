//
// Created by bontius on 03/05/16.
//

#include "tracking/phys/io/parabolaIo.h"

#include "tracking/phys/energyTerms/impl/gravityTerm.hpp"
#include "tracking/phys/parabola.h"
#include "tracking/phys/bundleWithPhysicsResult.h"
#include "picojson/picojson.h"
#include "picojson/picojsonUtil.h"
#include <string>
#include <fstream>
#include <iostream>

namespace tracking {
  namespace bundle_physics {
    namespace io {

      int writeInitialization(const BundleWithPhysicsResult &result, const std::string &path, const double fps) {
          //typedef Cuboid::Scalar Scalar;

          std::ofstream   f(path);
          picojson::array array;

          /// common for all parabolas
          {
              picojson::object object;

              // (1) gravity
              {
                  Scalar a(result.a);
                  if (result.gravity.norm() > kSmallDiff) {
                      Scalar _a = GravityCostFunctor::getAFromGravity(result.gravity.data(), fps);
                      if (std::abs(a - _a) > kSmallDiff) {
                          std::cerr << "a was " << a << ", and from gravity we get: " << _a << std::endl;
                          throw new WriteInitialization_GravityScaleDiscrepancyException("");
                      }
                  }
                  object["a"] = picojson::value(static_cast<double>(a));
              } //...gravity

              // (2) rot_x, rot_y1
              {
                  object["rot_x"]  = picojson::value(static_cast<double>(result.rotX));
                  object["rot_y1"] = picojson::value(static_cast<double>(result.rotY1));
              }

              // (3) collTime
              if (result.collTimes.size()) {
                  picojson::object jCollTimes;
                  for (auto const &pair : result.collTimes) {
                      jCollTimes[std::to_string(pair.first)] = picojson::value(static_cast<double>(pair.second));
                  }
                  object["collTimes"] = picojson::value(jCollTimes);
              }
//              if (result.collTimes.size() > 1) {
//                  std::cerr << "[" << __func__ << "] " << "not prepared for more collisions!" << std::endl;
//                  throw new std::runtime_error("");
//              } else if (result.collTimes.size()) {
//                  picojson::object jCollTimes;
//                  jCollTimes[]
//                  object["collTimes"] = picojson::value(picojson::array{picojson::value(static_cast<double>(result.collTimes.at(0)))});
//              }

              array.emplace_back(object);
          } //...parabolas

          // cuboids
          if (result.cuboids) {
              //CuboidId cuboidId( 0 );
              for (auto const &cuboidIdAndParts : result.parabolas) {
                  const CuboidId &cuboidId = cuboidIdAndParts.first;

                  picojson::object cuboidObject;
                  cuboidObject["cuboidId"] = picojson::value(static_cast<double>(cuboidId));

                  for (auto const &partIdAndParabola : cuboidIdAndParts.second) {
                      const PartId   &partId   = partIdAndParabola.first;
                      const Parabola &parabola = partIdAndParabola.second;
                      const CollId   collId    = std::max(0, partId - 1);

                      picojson::object partObject;
                      partObject["b"]      = picojson::value(parabola.b);
                      partObject["s"]      = picojson::value(parabola.s);
                      partObject["rot_y0"] = picojson::value(parabola.rotY0);
                      partObject["transl"] = picojson::array3ToPicojsonObject(parabola.getTranslation());

                      if (result.collisions.size() > 1)
                          throw new WriteInitialization_ASingleCollisionExpectedException("");
                      if (result.collisions.size()) {
                          if ((result.collisions.at(0).states.first.at(cuboidId).getPose().coeffs() - result.collisions.at(0).states.second.at(cuboidId).getPose().coeffs()).norm() > kSmallDiff)
                              throw new WriteInitialization_PoseDiscrepancyException("");
                          if (partId)
                              cuboidObject["pose"] = picojson::vector4ToPicojsonObject(result.collisions.at(collId).states.second.at(cuboidId).getPose().coeffs());
                          else
                              cuboidObject["pose"] = picojson::vector4ToPicojsonObject(result.collisions.at(collId).states.second.at(cuboidId).getPose().coeffs());
                      }

                      cuboidObject[std::to_string(partId)] = picojson::value(partObject);
                  }

                  array.emplace_back(cuboidObject);
              }
          } //...hasCuboids

          // write
          f << picojson::value(array).serialize();
          f.close();
          std::cout << "wrote to " << path << std::endl;

          return EXIT_SUCCESS;
      } //...writeInitialization

      bool parseShared(const picojson::object &object, BundleWithPhysicsResult &res) {
          //typedef Cuboid::Scalar Scalar;

          for (picojson::object::const_iterator entryIt = object.begin(); entryIt != object.end(); ++entryIt) {
              if (entryIt->first.compare("a") == 0)
                  res.a = static_cast<Scalar>(std::stof(entryIt->second.to_str().c_str()));
              else if (entryIt->first.compare("rot_x") == 0)
                  res.rotX = static_cast<Scalar>(std::stof(entryIt->second.to_str().c_str()));
              else if (entryIt->first.compare("rot_y1") == 0)
                  res.rotY1 = static_cast<Scalar>(std::stof(entryIt->second.to_str().c_str()));
              else if (entryIt->first.compare("collTimes") == 0 ) {
                  for (auto const& entry : entryIt->second.get<picojson::object>()) {
                      res.collTimes[std::atoi(entry.first.c_str())] = entry.second.get<double>();
                  }
              }
              else {
                  // intended, to distinguish between shared and free params
//                  std::cerr << "[" << __func__ << "] could not parse " << entryIt->first << std::endl;
                  return false;
              }
          }
          return true;
      } //...parseShared()

      bool parseParabola(const picojson::object &object, Parabola &parabola, Eigen::Vector3d &momentum) {
          for (picojson::object::const_iterator entryIt = object.begin(); entryIt != object.end(); ++entryIt) {
              if (entryIt->first.compare("b") == 0) {
                  parabola.b = std::stof(entryIt->second.to_str().c_str());
              }
              else if (entryIt->first.compare("s") == 0) {
                  parabola.s = std::stof(entryIt->second.to_str().c_str());
              }
              else if (entryIt->first.compare("rot_y0") == 0) {
                  parabola.rotY0 = std::stof(entryIt->second.to_str().c_str());
              }
              else if (entryIt->first.compare("transl") == 0) {
                  Eigen::Vector3d transl;
                  parseVector3(transl, entryIt->second.template get<picojson::object>());
                  parabola.setTranslation(transl.data());
              }
              else if (entryIt->first.compare("momentum") == 0) {
                  parseVector3(momentum, entryIt->second.template get<picojson::object>());
              }
              else {
                  std::cerr << "readInitialization: could not parse key " << entryIt->first << std::endl;
                  return false;
              }
          } //...keys

          return true;
      } //...parseParabola

      bool parseParabolas(const picojson::object &object, BundleWithPhysicsResult &res) {
          CuboidId                              cuboidId(0);
          bool                                  foundCuboidId(false);
          Parabolas::mapped_type                parabolas;
          QuaternionT::Coefficients             pose(QuaternionT::Coefficients::Zero()); // collisionPose
          BundleWithPhysicsResult::PartDynamics momenta;
          for (picojson::object::const_iterator entryIt = object.begin(); entryIt != object.end(); ++entryIt) {
              if (entryIt->first.compare("cuboidId") == 0) {
                  cuboidId      = std::atoi(entryIt->second.to_str().c_str());
                  foundCuboidId = true;
              }
              else if (entryIt->first.compare("pose") == 0) {
                  std::cout << "itt" << std::endl;
                  picojson::parseVector4(pose, entryIt->second.get<picojson::object>());
                  std::cout << "itt2" << std::endl;
              }
              else {
                  PartId partId(0);
                  try {
                      partId = std::stoi(entryIt->first.c_str());
                  }
                  catch (std::exception const &e) {
                      std::cout << "could not parse PartId, error : " << e.what() << std::endl;
                      throw;
                  }

                  Parabola        parabola;
                  Eigen::Vector3d momentum(Eigen::Vector3d::Zero());
                  parseParabola(entryIt->second.get<picojson::object>(), parabola, momentum);
                  if (parabolas.find(partId) != parabolas.end()) {
                      std::cerr << "duplicate partId in parabola parsing" << std::endl;
                      throw new ParseParabolas_DuplicatePartIdException("");
                  }
                  parabolas[partId] = parabola;
                  momenta[partId]   = momentum.cast<Scalar>();
              } //...if part
          } //...for each entry

          if (!foundCuboidId)
              throw new ParseParabolas_NoCuboidIdException("");
          if (res.parabolas.find(cuboidId) != res.parabolas.end())
              throw new ParseParabolas_DuplicateCuboidIdException("");
          res.parabolas[cuboidId] = parabolas;
          res.momenta[cuboidId]   = momenta;
          res.collisions[0].states.first[cuboidId].setPose(QuaternionT(pose(3), pose(0), pose(1), pose(2)));
          res.collisions[0].states.second[cuboidId].setPose(QuaternionT(pose(3), pose(0), pose(1), pose(2)));

          return true;
      } //...parseParabolas


      int readInitialization(BundleWithPhysicsResult &result, const std::string &inPath) {
          result.clear();

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
              //for (picojson::object::const_iterator entryIt = object.begin(); entryIt != object.end(); ++entryIt)
              {
                  bool parsed = false;

                  if (!parsed) {
                      parsed = parseShared(object, result);
                  }

                  if (!parsed) {
//                      std::cout << "calling parseParabolas on entry " << entryIndex << std::endl;
                      parsed = parseParabolas(object, result);
//                      std::cout << "called parseParabolas" << std::endl;
                  }

                  if (!parsed)
                      throw new ReadInitialization_CantParseObjectException("");
                  ++entryIndex;
              }
          } //...while entries in top array

          inFile.close();
          return EXIT_SUCCESS;
      } //...readInitialization()
    } //...ns io
  }//...ns bundle_physics
} //...ns tracking
