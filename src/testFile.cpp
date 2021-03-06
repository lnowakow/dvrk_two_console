//
// Created by dvrk-1804 on 2021-09-25.
//
#pragma once

#include "../include/stpJsonParser.h"
#include "../include/stpInputParser.h"
#include <iostream>

int main(int argc, char** argv) {

  InputParser input(argc, argv);


  const std::string& file_name = input.getCmdOption("-j");
  if (!file_name.empty()) {
    stpJsonParser parser(file_name);
    std::cout << parser.GetStringValue("Console1","MTMR", "name") << std::endl;
    std::cout << parser.GetMatrixValue("Console1","MTMR", "BASEFRAME").matrix() << std::endl;
  }
}