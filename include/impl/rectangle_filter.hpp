#ifndef RECTANGLE_FILTER_H
#define RECTANGLE_FILTER_H

#include "pcl/pcl_config.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/conditional_removal.h"

template <typename PointT>
RectangleFilter::RectangleFilter<PointT>::~RectangleFilter() {
  if (point_data_ != nullptr) {
    delete point_data;
    point_data_ = nullptr;
  }
}

template <typename PointT>
RectangleFilter::RectangleFilter<PointT>::evaluate (const PointT &point) const {
  return (true);
}

#endif
