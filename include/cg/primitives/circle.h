#pragma once

#include <cmath>

#include <cg/primitives/point.h>
#include <cg/primitives/vector.h>

template<class T>
T sqr(T x)
{
    return x * x;
}

template<class Scalar>
Scalar len_sqr(const cg::point_2t<Scalar> &a, const cg::point_2t<Scalar> &b)
{
    return sqr(a.x - b.x) + sqr(a.y - b.y);
}


namespace cg
{
    template<class Scalar>
    struct circle_2t;

    typedef circle_2t<double> circle_2;
    typedef circle_2t<float> circle_2f;

    template<class Scalar>
    struct circle_2t
    {
        Scalar r;
        point_2t<Scalar> center;

        circle_2t(const point_2t<Scalar> &a, const point_2t<Scalar> &b, const point_2t<Scalar> &c)
        {
            Scalar len_sqr_a = len_sqr(b, c);
            Scalar len_sqr_b = len_sqr(c, a);
            Scalar len_sqr_c = len_sqr(a, b);
            Scalar s = std::abs(a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)) / 2;
            r = std::sqrt(len_sqr_a * len_sqr_b * len_sqr_c) / s / 4;
            Scalar coef_a = len_sqr_a / (8 * sqr(s)) * ((a - b) * (a - c));
            Scalar coef_b = len_sqr_b / (8 * sqr(s)) * ((b - a) * (b - c));
            Scalar coef_c = len_sqr_c / (8 * sqr(s)) * ((c - a) * (c - b));
            center = point_2t<Scalar>(0, 0);
            center += coef_a * vector_2t<Scalar>(a.x, a.y);
            center += coef_b * vector_2t<Scalar>(b.x, b.y);
            center += coef_c * vector_2t<Scalar>(c.x, c.y);
        }
    };
}























