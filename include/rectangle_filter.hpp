#ifndef RECTANGLE_FILTER_H
#define RECTANGLE_FILTER_H

#pragma once
#include "pcl/pcl_config.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl/pcl_macros.h"

namespace RectangleFilter {
  template<typename PointT>
  class RectangleFilter : public pcl::ConditionBase<PointT>
  {
    using pcl::ConditionBase<PointT>::conditions_;
    using pcl::ConditionBase<PointT>::comparisons_;
    public:
      using Ptr = std::shared_ptr<RectangleFilter<PointT>>;
      using ConstPtr = std::shared_ptr<const RectangleFilter<PointT>>;

      RectangleFilter () :
        pcl::ConditionBase<PointT>() {
      }

    //~RectangleFilter()

      bool evaluate(const PointT &point) const override {
        return (true);
      }
  };

}

#define PCL_INSTANTIATE_RectangleFilter(T) template class PCL_EXPORTS RectangleFilter::RectangleFilter<T>;

#endif
