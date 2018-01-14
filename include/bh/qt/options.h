//==================================================
// options.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Feb 21, 2017
//==================================================

#pragma once

#include <QColor>
#include <boost/program_options.hpp>
#include <boost/throw_exception.hpp>
#include <boost/algorithm/string.hpp>
#include <type_traits>

namespace boost {

void validate(boost::any& v,
              const std::vector<std::string>& values,
              QColor* target_type, int);

inline void validate(boost::any& v,
                     const std::vector<std::string>& values,
                     QColor* target_type, int) {
  using namespace boost::program_options;

  // Make sure no previous assignment to 'v' was made.
  validators::check_first_occurrence(v);

  const std::string& s = validators::get_single_string(values);

  std::vector<std::string> split_vec;
  boost::split(split_vec, s, boost::is_any_of(" ,"), boost::token_compress_on);

  if (split_vec.size() != 3 && split_vec.size() != 4) {
    boost::throw_exception(validation_error(validation_error::invalid_option_value));
  }

  QColor color;
  color.setRedF(boost::lexical_cast<qreal>(split_vec[0]));
  color.setGreenF(boost::lexical_cast<qreal>(split_vec[1]));
  color.setBlueF(boost::lexical_cast<qreal>(split_vec[2]));
  if (split_vec.size() == 4) {
    color.setAlphaF(boost::lexical_cast<qreal>(split_vec[3]));
  }
  v = boost::any(color);
}

}
