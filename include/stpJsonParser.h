//
// Created by dvrk-1804 on 2021-09-25.
//

#ifndef DVRK_TWO_CONSOLE_LIBS_STPJSONPARSER_H_
#define DVRK_TWO_CONSOLE_LIBS_STPJSONPARSER_H_

#include <string>
#include <vector>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>


class stpJsonParser {
 public:
  stpJsonParser() {}
  stpJsonParser(std::string document) {
    openFile(document);
  }
  ~stpJsonParser() {}

  inline void openFile(std::string document) {
    std::ifstream  file(document, std::ifstream::binary);
    if (file.is_open()) {
      std::cout << "file: " << document << " is open\n";
      reader.parse(file, roots);
    } else {
      std::cout << "File failed to open\n";
    }
  }

  inline std::string GetStringValue(std::string structName) {
    std::string value = roots[structName].toStyledString();
    value.erase(std::remove(value.begin(), value.end(),'"'), value.end());
    value.erase(std::remove(value.begin(), value.end(),'\n'), value.end());
    return value;
  }

  inline std::string GetStringValue(std::string structName, std::string topicName) {
    std::string value = roots[structName][topicName].toStyledString();
    value.erase(std::remove(value.begin(), value.end(),'"'), value.end());
    value.erase(std::remove(value.begin(), value.end(),'\n'), value.end());
    return value;
  }

  inline std::string GetStringValue(std::string structName, std::string paramName, std::string fieldValue) {
    std::string value = roots[structName][paramName][fieldValue].toStyledString();
    value.erase(std::remove(value.begin(), value.end(),'"'), value.end());
    value.erase(std::remove(value.begin(), value.end(),'\n'), value.end());
    return value;
  }

  inline Eigen::Isometry3d GetMatrixValue(std::string structName, std::string paramName, std::string baseframe) {
    //std::cout << "Baseframe shows: " << roots[structName][paramName][baseframe] << '\n';

    Eigen::Matrix3d rotation;
    rotation <<
      (double)std::stof(roots[structName][paramName][baseframe][0][0].toStyledString()),
      (double)std::stof(roots[structName][paramName][baseframe][0][1].toStyledString()),
      (double)std::stof(roots[structName][paramName][baseframe][0][2].toStyledString()),
      (double)std::stof(roots[structName][paramName][baseframe][1][0].toStyledString()),
      (double)std::stof(roots[structName][paramName][baseframe][1][1].toStyledString()),
      (double)std::stof(roots[structName][paramName][baseframe][1][2].toStyledString()),
      (double)std::stof(roots[structName][paramName][baseframe][2][0].toStyledString()),
      (double)std::stof(roots[structName][paramName][baseframe][2][1].toStyledString()),
      (double)std::stof(roots[structName][paramName][baseframe][2][2].toStyledString());
    //std::cout << rotation << '\n';

    Eigen::Matrix3Xd translation(3,1);
    translation <<
      (double)std::stof(roots[structName][paramName][baseframe][0][3].toStyledString()),
      (double)std::stof(roots[structName][paramName][baseframe][1][3].toStyledString()),
      (double)std::stof(roots[structName][paramName][baseframe][2][3].toStyledString());
    //std::cout << translation << '\n';

    Eigen::Isometry3d Matrix;
    Matrix.matrix().block(0,0,3,3) = rotation;
    Matrix.matrix().block(0,3,3,1) = translation;


    return Matrix;
  }



 private:
  Json::Reader reader;
  Json::Value roots;


};


#endif //DVRK_TWO_CONSOLE_LIBS_STPJSONPARSER_H_
