#ifndef __POINTCLOUD_HH__
#define __POINTCLOUD_HH__

#include <vector>
#include <base/time.h>
#include <base/eigen.h>


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

