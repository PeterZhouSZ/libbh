//==================================================
// hdf5_utils.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Oct 10, 2017
//==================================================
#pragma once

#include "common.h"
#include <vector>
#include <H5Cpp.h>

namespace bh {

class HDF5Utilities {
public:

  using size_t = std::size_t;

  static std::vector<size_t> readHDF5DatasetDimensions(const H5::DataSet& dataset) {
    const H5::DataSpace dataspace = dataset.getSpace();
    const int rank = dataspace.getSimpleExtentNdims();
    std::vector<hsize_t> dims_out(rank);
    const int ndims = dataspace.getSimpleExtentDims(dims_out.data(), nullptr);
//    cout << "Dataset has rank " << rank << endl;
//    for (int i = 0; i < ndims; ++i) {
//      cout << "Dimension " << i << ": " << (unsigned long) (dims_out[i]) << endl;
//    }
    BH_ASSERT(rank == ndims);
    std::vector<size_t> dimensions(ndims);
    std::copy(dims_out.begin(), dims_out.end(), dimensions.begin());
    return dimensions;
  }

  template <typename T>
  static std::vector<T> readHDF5DatasetAsVector(const H5::DataSet& dataset, const H5::DataType& data_type) {
    const std::vector<std::size_t> dimensions = readHDF5DatasetDimensions(dataset);
    const size_t total_size = std::accumulate(dimensions.begin(), dimensions.end(), (size_t)1,
                                              [](const size_t a, const size_t b) { return a * b; });
    std::vector<T> values(total_size);
    dataset.read(values.data(), data_type);
    return values;
  }

};

}
