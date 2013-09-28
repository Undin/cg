#include <gtest/gtest.h>

#include <boost/assign/list_of.hpp>

#include <vector>
#include <iterator>
#include <algorithm>

#include <cg/primitives/point.h>
#include <cg/scalar_product/simplify.h>

#include "random_utils.h"

bool is_ok(const std::vector<cg::point_2> &original, const std::vector<cg::point_2> &result, double eps)
{
    auto begin = original.begin();
    auto end = original.end();

    for (int i = 1; i < result.size(); i++)
    {
        auto res = std::find(begin, end, result[i]);
        if (res == end)
        {
            return false;
        }
        for (auto it = begin; it != res; it++)
        {
            if (cg::dist(result[i - 1], result[i], *it) >= eps)
            {
                return false;
            }
        }
        begin = res;
    }
    return true;
}

using cg::point_2;

TEST(simplify, simple1)
{
    std::vector<point_2> pts = boost::assign::list_of(point_2(0, 0))
                                                     (point_2(2, 3))
                                                     (point_2(0, 100));
    std::vector<point_2> res;

    for (double eps = 0.1; eps < 3; eps += 0.1)
    {
        res.clear();
        cg::simplify(pts.begin(), pts.end(), eps, std::back_inserter(res));
        EXPECT_TRUE(is_ok(pts, res, eps));
    }
}

TEST(simplify, simple2)
{
    std::vector<point_2> pts = boost::assign::list_of(point_2(1, 1))
                                                     (point_2(50, 2))
                                                     (point_2(100, 1));
    std::vector<point_2> res;

    for (double eps = 0.1; eps < 3; eps += 0.1)
    {
        res.clear();
        cg::simplify(pts.begin(), pts.end(), eps, std::back_inserter(res));
        EXPECT_TRUE(is_ok(pts, res, eps));
    }
}

TEST(simplify, simple3)
{
    std::vector<point_2> pts = boost::assign::list_of(point_2(0, 0))
                                                     (point_2(12, 4))
                                                     (point_2(5, 7))
                                                     (point_2(123, 98))
                                                     (point_2(100, 87))
                                                     (point_2(34, -10))
                                                     (point_2(-1, 23))
                                                     (point_2(-9, -7))
                                                     (point_2(-10, -8));
    std::vector<point_2> res;

    for (double eps = 0.1; eps < 10; eps += 0.1)
    {
        res.clear();
        cg::simplify(pts.begin(), pts.end(), eps, std::back_inserter(res));
        EXPECT_TRUE(is_ok(pts, res, eps));
    }
}

TEST(simplify, uniform1)
{
    std::vector<point_2> pts = uniform_points(100000);
    std::vector<point_2> res;
    for (double eps = 0.1; eps < 20; eps += 0.5)
    {
        res.clear();
        cg::simplify(pts.begin(), pts.end(), eps, std::back_inserter(res));
        EXPECT_TRUE(is_ok(pts, res, eps));
    }
}
