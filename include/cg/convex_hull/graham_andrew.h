#pragma once

#include <utility>
#include <algorithm>
#include <functional>
#include "cg/operations/orientation.h"

namespace cg
{
    template <class RunIter>
    RunIter find_convex_hull_point(RunIter begin, RunIter end, typename std::iterator_traits<RunIter>::value_type const &next_element)
    {
        if (end - begin < 2)
        {
            return begin + 1;
        }
        RunIter res = begin + 2;
        for (RunIter it = begin + 2; it != end; it++)
        {
            while (res - 1 != begin && orientation(*(res - 2), *(res - 1), *it) != CG_LEFT)
            {
                res--;
            }
            std::swap(*res, *it);
            res++;
        }
        while (res - 1 != begin && orientation(*(res - 2), *(res - 1), next_element) != CG_LEFT)
        {
            res--;
        }
        return res;
    }

    template <class RunIter>
    RunIter graham_andrew_hull(RunIter begin, RunIter end)
    {
        if (begin == end || begin + 1 == end || begin + 2 == end)
        {
            return end;
        }
        std::pair<RunIter, RunIter> min_max = std::minmax_element(begin, end);
        point_2 min_point = *min_max.first;
        point_2 max_point = *min_max.second;
        std::swap(*min_max.first, *begin);
        std::swap(*min_max.second, *(end - 1));
        RunIter bound = std::partition(begin + 1, end - 1, [min_point, max_point](const point_2 &a)
        {
            return orientation(min_point, max_point, a) == CG_RIGHT;
        });
        std::sort(begin, bound);
        std::sort(bound, end, std::greater<point_2>());

        RunIter first_last = find_convex_hull_point(begin, bound, *bound);
        RunIter second_last = find_convex_hull_point(bound, end, *begin);
        std::swap_ranges(bound, second_last, first_last);
        return first_last + (second_last - bound);
    }
}
