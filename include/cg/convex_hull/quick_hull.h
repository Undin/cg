#pragma once

#include <cg/primitives/point.h>
#include <cg/operations/orientation.h>
#include <algorithm>
#include <utility>

namespace cg
{
    double triangle_sqr(point_2 const & a, point_2 const & b, point_2 const & c)
    {
        double l = (b.x - a.x) * (c.y - a.y);
        double r = (b.y - a.y) * (c.x - a.x);
        double res = l - r;
        return std::fabs(res);
    }

    template <class RanIter>
    RanIter build_part(RanIter begin, RanIter end, typename std::iterator_traits<RanIter>::value_type last_point)
    {
        if (begin + 1 == end)
        {
            return end;
        }

        RanIter highest_point_it = std::max_element(begin, end, [begin, last_point](const point_2 &largest, const point_2 &first)
        {
            return triangle_sqr(*begin, last_point, largest) < triangle_sqr(*begin, last_point, first);
        });

        point_2 highest_point = *highest_point_it;

        if (orientation(*begin, last_point, highest_point) == CG_COLLINEAR)
        {
            return begin + 1;
        }
        std::swap(*(begin + 1), *highest_point_it);

        RanIter first = std::partition(begin + 2, end, [begin, highest_point](const point_2 &point)
        {
            return orientation(*begin, highest_point, point) == CG_RIGHT;
        });

        RanIter second = std::partition(first, end, [highest_point, last_point](const point_2 &point)
        {
            return orientation(highest_point, last_point, point) == CG_RIGHT;
        });

        std::swap(*(begin + 1), *(first - 1));

        RanIter first_end = build_part(begin, first - 1, highest_point);
        RanIter second_end = build_part(first - 1, second, last_point);
        return std::swap_ranges(first - 1, second_end, first_end);
    }

    template <class RanIter>
    RanIter quick_hull(RanIter begin, RanIter end)
    {
        if (begin == end || begin + 1 == end || begin + 2 == end)
        {
            return end;
        }

        std::pair<RanIter, RanIter> min_max = std::minmax_element(begin, end);
        point_2 min_point = *min_max.first;
        point_2 max_point = *min_max.second;
        std::swap(*min_max.first, *begin);
        std::swap(*min_max.second, *(end - 1));
        RanIter bound = std::partition(begin + 1, end - 1, [min_point, max_point](const point_2 &a)
        {
            return orientation(min_point, max_point, a) == CG_RIGHT;
        });

        std::swap(*(end - 1), *bound);
        RanIter first = build_part(begin, bound, max_point);
        RanIter second = build_part(bound, end, min_point);
        return std::swap_ranges(bound, second, first);
    }
}
