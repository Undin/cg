#pragma once

#include <utility>
#include <algorithm>
#include <functional>
#include <cg/operations/orientation.h>
#include <cg/convex_hull/graham.h>

namespace cg
{
    template <class RunIter>
    RunIter graham_andrew_hull(RunIter begin, RunIter end)
    {
        if (begin == end)
        {
            return end;
        }
        std::pair<RunIter, RunIter> min_max = std::minmax_element(begin, end);
        point_2 min_point = *min_max.first;
        point_2 max_point = *min_max.second;
        if (min_point == max_point)
        {
            return ++begin;
        }
        std::iter_swap(min_max.first, begin);
        std::iter_swap(min_max.second, end - 1);
        RunIter bound = std::partition(begin + 1, end - 1, [&min_point, &max_point](const point_2 &a)
        {
            return orientation(min_point, max_point, a) != CG_LEFT;
        });

        std::sort(begin, bound);
        std::sort(bound, end, std::greater<point_2>());

        return contour_graham_hull(begin, end);
    }
}
