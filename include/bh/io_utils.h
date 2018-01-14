//==================================================
// io_utils.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 01.12.17
//==================================================
#pragma once

#include "string_utils.h"

namespace bh {

template<typename Stream>
Stream& getNonCommentLine(Stream &stream, std::string &line, const std::string &comment_chars) {
  while (std::getline(stream, line)) {
    bh::trim(line);
    if (!line.empty()) {
      if (!boost::is_any_of(comment_chars)(line[0])) {
        return stream;
      }
    }
  }
  return stream;
}

template<typename Stream>
std::string getNonCommentLine(Stream &stream, const std::string &comment_chars) {
  std::string line;
  while (std::getline(stream, line)) {
    bh::trim(line);
    if (!line.empty()) {
      if (boost::is_any_of(comment_chars)(line[0])) {
        return line;
      }
    }
  }
  return std::string();
}

}