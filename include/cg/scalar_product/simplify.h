#pragma once

#include <cmath>
#include <algorithm>

#include <boost/utility.hpp>

#include <cg/operations/orientation.h>
#include <cg/primitives/point.h>
#include <cg/io/point.h>

namespace cg
{

template <class BidIter, class OutIter>
OutIter douglas_peucker(BidIter p, BidIter q, double eps, OutIter out)
{
    if (boost::next(p) == q)
    {
        *out++ = *p;
        return out;
    }
    BidIter max_point = std::max_element(boost::next(p), q, [p, q](typename std::iterator_traits<BidIter>::value_type largest,
                                                                   typename std::iterator_traits<BidIter>::value_type current)
    {
        return dist(*p, *q, current) > dist(*p, *q, largest);
    });

    if (dist(*p, *q, *max_point) < eps)
    {
        *out++ = *p;
        return out;
    }
    else
    {
        out = douglas_peucker(p, max_point, eps, out);
        return douglas_peucker(max_point, q, eps, out);
    }
}

template <class BidIter, class OutIter>
OutIter simplify(BidIter p, BidIter q, double eps, OutIter out)
{
    if (boost::next(p) != q)
    {
        out = douglas_peucker(p, boost::prior(q), eps, out);
    }
    *out++ = *boost::prior(q);
    return out;
}

double dist(point_2 a, point_2 b)
{
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

double dist(point_2 a, point_2 b, point_2 c)
{
    double pr = ((c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y)) / dist(a, b);
    if (pr < 0)
    {
        return dist(a, c);
    }
    if (pr > dist(a, b))
    {
        return dist(b, c);
    }
    return std::abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) / dist(a, b);
}
}
