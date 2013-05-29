#ifndef BASE_POINTCLOUD_HPP
#define BASE_POINTCLOUD_HPP

#include <vector>
#include <base/Time.hpp>
#include <base/Point.hpp>

namespace base {

  typedef base::Vector3d    Point;

  namespace samples {
  struct Pointcloud
  {
    Time time;

    std::vector<base::Point> points;
  };


}}


#endif

