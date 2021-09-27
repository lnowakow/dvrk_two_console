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
      std::cout << "file is open\n";
      reader.parse(file, roots);
    }
  }

  inline std::string GetStringValue(std::string structName, std::string topicName) {
    std::string value = roots[structName][topicName].toStyledString();
    value.erase(std::remove(value.begin(), value.end(),'"'), value.end());
    value.erase(std::remove(value.begin(), value.end(),'\n'), value.end());
    return value;
  }

 private:
  Json::Reader reader;
  Json::Value roots;

};


#endif //DVRK_TWO_CONSOLE_LIBS_STPJSONPARSER_H_
