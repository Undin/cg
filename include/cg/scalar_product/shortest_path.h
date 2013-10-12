#pragma once

#include <vector>
#include <map>
#include <algorithm>
#include <iterator>
#include <utility>
#include <cmath>

#include <boost/optional.hpp>

#include <cg/primitives/point.h>
#include <cg/primitives/contour.h>
#include <cg/primitives/segment.h>

#include <cg/scalar_product/visibility_graph.h>

namespace cg
{


//class rotation_tree
//{
//private:
//    struct node
//    {
//        point_2 point;
//        point_2 *parent;
//        point_2 *left_brother;
//        point_2 *right_brother;
//        point_2 *rightmost_son;

//        node(point_2 point,
//             point_2 *parent = nullptr, point_2 *left_brother = nullptr,
//             point_2 *right_brother = nullptr, point_2 *rightmost_son = nullptr) :
//                                                                                    point(point),
//                                                                                    parent(parent),
//                                                                                    left_brother(left_brother),
//                                                                                    right_brother(right_brother),
//                                                                                    rightmost_son(rightmost_son)
//        {}

//        ~node()
//        {
//            if (rightmost_son != nullptr)
//            {
//                delete rightmost_son;
//            }
//            if (left_brother != nullptr)
//            {
//                delete left_brother;
//            }
//        }
//    };

//    point_2 *root;


//};

double len(const cg::segment_2 &a)
{
    return std::sqrt((a[0].x - a[1].x) * (a[0].x - a[1].x) + (a[0].y - a[1].y) * (a[0].y - a[1].y));
}

template <class FwdIter, class OutIter>
OutIter shortest_path(FwdIter begin, FwdIter end, const point_2 &s, const point_2 &t, OutIter out)
{
    std::vector<contour_2> contours;
    std::copy(begin, end, std::back_inserter(contours));
    contours.push_back(contour_2(std::vector<point_2>({s})));
    contours.push_back(contour_2(std::vector<point_2>({t})));
    std::map<cg::point_2, int> m;
    std::vector<point_2> rm;
    int count = 0;
    m[s] = count++;
    rm.push_back(s);
    if (m.find(t) == m.end())
    {
        m[t] = count++;
        rm.push_back(t);
    }
    for (FwdIter it = begin; it != end; ++it)
    {
        contour_2 contour = *it;
        for (auto itr = contour.begin(); itr != contour.end(); ++itr)
        {
            if (m.find(*itr) == m.end())
            {
                m[*itr] = count++;
                rm.push_back(*itr);
            }
        }
    }
    std::vector<segment_2> res;
    visibility_graph_naive_impl(contours.begin(), contours.end(), std::back_inserter(res));
    std::vector<std::vector<std::pair<int, double> > > edges(count);
    for (int i = 0; i < res.size(); i++)
    {
        int u = m[res[i][0]];
        int v = m[res[i][1]];
        double dist = len(res[i]);
        edges[u].push_back(std::make_pair(v, dist));
        edges[v].push_back(std::make_pair(u, dist));
    }
    double inf = std::numeric_limits<double>::infinity();
    std::vector<double> d(count, inf);
    std::vector<bool> used(count, false);
    std::vector<int> parent(count);
    d[0] = 0;
    boost::optional<int> u = 0;
    while (u)
    {
        used[*u] = true;
        for (int i = 0; i < edges[*u].size(); i++)
        {
            int v = edges[*u][i].first;
            double w = edges[*u][i].second;
            if (!used[v] && d[v] > d[*u] + w)
            {
                d[v] = d[*u] + w;
                parent[v] = *u;
            }
        }
        u = boost::none;
        for (int i = 0; i < count; i++)
        {
            if (!used[i] && d[i] < inf && (!u || d[*u] > d[i]))
            {
                u = i;
            }
        }
    }

    if (d[m[t]] < inf)
    {
        int current = m[t];
        std::vector<int> path = {current};
        while (current != 0)
        {
            current = parent[current];
            path.push_back(current);
        }

        for (int i = path.size() - 1; i >= 0; i--)
        {
            out++ = rm[path[i]];
        }
    }
    return out;
}


}
