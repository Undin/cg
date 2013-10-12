#pragma once

#include <boost/utility.hpp>

#include <cg/primitives/point.h>
#include <cg/primitives/segment.h>
#include <cg/primitives/contour.h>

#include <cg/operations/contains/segment_point.h>
#include <cg/operations/has_intersection/segment_segment.h>
#include <cg/operations/orientation.h>

#include <cg/io/segment.h>

namespace cg {

bool is_out_contour(const cg::point_2 &a , const cg::point_2 &b ,const cg::point_2 &c ,const cg::point_2 &d)
{
    return (orientation(a, b, c) != cg::CG_RIGHT && (orientation(a, b, d) != cg::CG_LEFT || orientation(b, c, d) != cg::CG_LEFT)) ||
           (orientation(a, b, c) == cg::CG_RIGHT && (orientation(a, b, d) != cg::CG_LEFT && orientation(b, c, d) != cg::CG_LEFT));
}

bool intersection(const cg::segment_2 &first, const cg::segment_2 &second)
{
    if (!has_intersection(first, second))
    {
        return false;
    }
    else
    {
        bool first_constains_second0 = contains(first, second[0]);
        bool first_constains_second1 = contains(first, second[1]);
        bool second_constains_first0 = contains(second, first[0]);
        bool second_constains_first1 = contains(second, first[1]);
        if (first[0] == second[0] && !first_constains_second1 && !second_constains_first1 ||
            first[0] == second[1] && !first_constains_second0 && !second_constains_first1 ||
            first[1] == second[0] && !first_constains_second1 && !second_constains_first0 ||
            first[1] == second[1] && !first_constains_second0 && !second_constains_first0)
        {
            return false;
        }
        return true;
    }
}

bool have_not_intersection(const cg::segment_2 &segment, const cg::contour_2 &contour)
{
    if (contour.size() == 0)
    {
        return true;
    }
    if (contour.size() == 1)
    {
        if (contains(segment, contour[0]))
        {
            return segment[0] == contour[0] || segment[1] == contour[0];
        }
        return true;
    }
    for (auto it = contour.begin(); it != contour.end(); ++it)
    {
        auto p = contour.circulator(it);
        segment_2 second_segment(*p, *boost::next(p));
        if (max(segment) != max(second_segment) || min(segment) != min(second_segment))
        {
            if (intersection(segment, second_segment))
            {
                return false;
            }
        }
    }
    return true;
}

template <class FwdIter, class OutIter>
OutIter for_contour(const cg::contour_2 &contour, FwdIter begin, FwdIter end, OutIter out)
{
    for (int i = 0; i < contour.size(); ++i)
    {
        auto b = contour.circulator(contour.begin() + i);
        auto a = boost::prior(b);
        auto c = boost::next(b);
        for (int j = i + 1; j < contour.size(); ++j)
        {
            bool has_not_intersect = true;
            auto c1 = contour.circulator(contour.begin() + i);
            auto c2 = contour.circulator(contour.begin() + j);
            if (!((orientation(*boost::prior(c1), *c1, contour[j]) == cg::CG_RIGHT && orientation(*c1, *boost::next(c1), contour[j]) == cg::CG_RIGHT) ||
                (orientation(*boost::prior(c2), *c2, contour[i]) == cg::CG_RIGHT && orientation(*c2, *boost::next(c2), contour[i]) == cg::CG_RIGHT)))
            {
                segment_2 segment(contour[i], contour[j]);
                for (FwdIter it = begin; it != end && has_not_intersect; ++it)
                {
                    has_not_intersect &= have_not_intersection(segment, *it);
                }
                if (has_not_intersect && is_out_contour(*a, *b, *c, contour[j]))
                {
                    out++ = segment_2(*b, contour[j]);
                }
            }
        }
    }
    return out;
}

template <class FwdIter, class OutIter>
OutIter visibility_graph_naive_impl(FwdIter begin, FwdIter end, OutIter out)
{
    for (FwdIter i = begin; i != end; ++i)
    {
        contour_2 contour_p = *i;
        out = for_contour(contour_p, begin, end, out);
        for (auto p = contour_p.begin(); p != contour_p.end(); ++p)
        {
            for (FwdIter j = boost::next(i); j != end; ++j)
            {
                contour_2 contour_q = *j;
                for (auto q = contour_q.begin(); q != contour_q.end(); ++q)
                {
                    auto c1 = contour_p.circulator(p);
                    auto c2 = contour_q.circulator(q);
                    if (!((orientation(*boost::prior(c1), *c1, *q) == cg::CG_RIGHT && orientation(*c1, *boost::next(c1), *q) == cg::CG_RIGHT) ||
                        (orientation(*boost::prior(c2), *c2, *p) == cg::CG_RIGHT && orientation(*c2, *boost::next(c2), *p) == cg::CG_RIGHT)))
                    {
                        segment_2 segment(*p, *q);
                        bool has_not_interection = true;
                        for (FwdIter k = begin; k != end && has_not_interection; ++k)
                        {
                            has_not_interection &= have_not_intersection(segment, *k);
                        }
                        if (has_not_interection)
                        {
                            out++ = segment;
                        }
                    }
                }
            }
        }
    }
    return out;
}
}
