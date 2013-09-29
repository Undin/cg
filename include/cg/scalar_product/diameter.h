#pragma once

#include <utility>
#include <algorithm>
#include <vector>
#include <iterator>
#include <iostream>


#include <boost/utility.hpp>

#include <cg/io/point.h>
#include <cg/primitives/point.h>
#include <cg/operations/orientation.h>
#include <cg/convex_hull/quick_hull.h>

namespace cg {

template <class Scalar>
Scalar sqr_of_distance(point_2 const &a, point_2 const &b)
{
    return (Scalar(a.x) - b.x) * (Scalar(a.x) - b.x) + (Scalar(a.y) - b.y) * (Scalar(a.y) - b.y);
}

struct pred_dist_d
{
   boost::optional<bool> operator() (point_2 const &a, point_2 const &b, point_2 const &c, point_2 const &d)  const
   {
      double dist1 = sqr_of_distance<double>(a, b);
      double dist2 = sqr_of_distance<double>(c, d);
      double res = dist1 - dist2;
      double eps = (dist1 + dist2) * 16 * std::numeric_limits<double>::epsilon();

      if (res > eps)
         return false;

      if (res < -eps)
         return true;

      return boost::none;
   }
};

struct pred_dist_i
{
   boost::optional<bool> operator() (point_2 const &a, point_2 const &b, point_2 const &c, point_2 const &d) const
   {
      typedef boost::numeric::interval_lib::unprotect<boost::numeric::interval<double> >::type interval;

      boost::numeric::interval<double>::traits_type::rounding _;
      interval res = sqr_of_distance<interval>(a, b) - sqr_of_distance<interval>(c, d);

      if (res.lower() > 0)
         return false;

      if (res.upper() < 0)
         return true;

      return boost::none;
   }
};

struct pred_dist_r
{
   boost::optional<bool> operator() (point_2 const &a, point_2 const &b, point_2 const &c, point_2 const &d) const
   {
      mpq_class res = sqr_of_distance<mpq_class>(a, b) - sqr_of_distance<mpq_class>(c, d);
      int cres = cmp(res, 0);

      if (cres < 0)
      {
         return true;
      }
      return false;
   }
};

inline bool pred_dist(point_2 const &a, point_2 const &b, point_2 const &c, point_2 const &d)
{
   if (boost::optional<bool> v = pred_dist_d()(a, b, c, d))
      return *v;

   if (boost::optional<bool> v = pred_dist_i()(a, b, c, d))
      return *v;

   return *pred_dist_r()(a, b, c, d);
}

using std::cout;
using std::endl;

template <class FwdIter>
std::pair<point_2, point_2> diameter(FwdIter begin, FwdIter end)
{
    std::vector<point_2> points;
    std::copy(begin, end, std::back_inserter(points));
    std::vector<point_2>::iterator iter = quick_hull(points.begin(), points.end());

    if (points.begin() + 1 == iter)
    {
        return std::make_pair(points[0], points[0]);
    }
    if (points.begin() + 2 == iter)
    {
        return std::make_pair(points[0], points[1]);
    }

    auto left = std::min_element(points.begin(), iter);
    auto right = std::max_element(points.begin(), iter);

    std::vector<point_2>::iterator first, second;
    first = left;
    second = right;
    auto result = std::make_pair(*first, *second);
    bool first_end = false;
    bool second_end = false;
    while (!first_end && !second_end)
    {
        auto first_next = boost::next(first) != iter ? boost::next(first) : points.begin();
        auto second_next = boost::next(second) != iter ? boost::next(second) : points.begin();
        if (pred(*first, *first_next, *second, *second_next) == CG_RIGHT)
        {
            second = second_next;
            second_end |= second == left;

        }
        else
        {
            first = first_next;
            first_end |= first == right;
        }
        if (pred_dist(result.first, result.second, *first, *second))
        {
            result = std::make_pair(*first, *second);
        }
    }
    return result;
}
}
