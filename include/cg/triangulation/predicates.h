#pragma once

#include <cmath>
#include <array>
#include <iostream>

#include <boost/optional.hpp>

#include <gmpxx.h>

#include <cg/primitives/point.h>
#include <cg/operations/orientation.h>
#include <cg/triangulation/cell_structure.h>

using namespace std;

namespace cg
{
    enum class CircleContent
    {
        IN_CIRCLE,
        ON_CIRCLE,
        OUT_CIRCLE
    };

    template<class Scalar>
    Scalar pointSqr(const point_2 &p)
    {
        return ((Scalar) p.x) * p.x + ((Scalar) p.y) * p.y;
    }

    template<class Scalar>
    Scalar determinant(const std::array<Vertex, 3> &v, const Vertex &d)
    {
        Scalar result = 0;
        for (int i = 0; i < 3; i++)
        {
            result += ((Scalar) v[i]->point.x - d->point.x) *
                      ((Scalar) v[next(i)]->point.y - d->point.y) *
                      (pointSqr<Scalar>(v[prev(i)]->point) - pointSqr<Scalar>(d->point)) -
                      ((Scalar) v[i]->point.x - d->point.x) *
                      ((Scalar) v[prev(i)]->point.y - d->point.y) *
                      (pointSqr<Scalar>(v[next(i)]->point) - pointSqr<Scalar>(d->point));
        }
        return result;
    }

    struct inCircleD
    {
        boost::optional<CircleContent> operator()(const std::array<Vertex, 3> &v, const Vertex &d) const
        {
            if (d->inf)
            {
                return CircleContent::OUT_CIRCLE;
            }
            bool infFace = false;
            for (int i = 0; i < 3; i++)
            {
                if (v[i]->inf)
                {
                    infFace = true;
                    if (orientation(v[next(i)]->point, v[prev(i)]->point, d->point) == cg::CG_LEFT)
                    {
                         return CircleContent::IN_CIRCLE;
                    }
                }
            }
            if (infFace)
            {
                return CircleContent::OUT_CIRCLE;
            }

            double result = determinant<double>(v, d);
            double resultAbs = 0;
            for (int i = 0; i < 3; i++)
            {
                resultAbs += fabs((v[i]->point.x - d->point.x) *
                                  (v[next(i)]->point.y - d->point.y) *
                                  (pointSqr<double>(v[prev(i)]->point) - pointSqr<double>(d->point))) +
                             fabs((v[i]->point.x - d->point.x) *
                                  (v[prev(i)]->point.y - d->point.y) *
                                  (pointSqr<double>(v[next(i)]->point) - pointSqr<double>(d->point)));
            }
            double eps = resultAbs * 16 * std::numeric_limits<double>::epsilon();
            if (result > eps)
            {
                return CircleContent::IN_CIRCLE;
            }
            if (result < -eps)
            {
                return CircleContent::OUT_CIRCLE;
            }
            return boost::none;
        }
    };

    struct inCircleI
    {
        typedef boost::numeric::interval_lib::unprotect<boost::numeric::interval<double> >::type interval;

        boost::numeric::interval<double>::traits_type::rounding _;

        boost::optional<CircleContent> operator()(const std::array<Vertex, 3> &v, const Vertex &d)
        {
            interval result = determinant<interval>(v, d);
            if (result.lower() > 0)
            {
                return CircleContent::IN_CIRCLE;
            }
            if (result.upper() < 0)
            {
                return CircleContent::OUT_CIRCLE;
            }
            return boost::none;
        }
    };

    struct inCircleR
    {
        boost::optional<CircleContent> operator()(const std::array<Vertex, 3> &v, const Vertex &d)
        {
            mpq_class result = determinant<mpq_class>(v, d);
            if (result > 0)
            {
                return CircleContent::IN_CIRCLE;
            }
            if (result < 0)
            {
                return CircleContent::OUT_CIRCLE;
            }
            return CircleContent::ON_CIRCLE;
        }
    };

    CircleContent inCircle(const Vertex &a, const Vertex &b, const Vertex &c, const Vertex &d)
    {
        std::array<Vertex, 3> v;
        v[0] = a;
        v[1] = b;
        v[2] = c;
        if (boost::optional<CircleContent> res = inCircleD()(v, d))
           return *res;

        if (boost::optional<CircleContent> res = inCircleI()(v, d))
           return *res;

        return *inCircleR()(v, d);
    }

    bool badEdge(const Edge &e)
    {
        Vertex a = e->first_vertex;
        Vertex b = e->second_vertex;
        Vertex c = e->next->second_vertex;
        Vertex d = e->twin->next->second_vertex;
        return inCircle(a, b, c, d) == CircleContent::IN_CIRCLE;
    }

    bool inFace(const Face &f, const Vertex &v)
    {
        bool result = true;
        Edge e = f->edge;
        for (int i = 0; i < 3 && result; i++, e = e->next)
        {
            result &= (e->first_vertex->inf  ||
                       e->second_vertex->inf ||
                       orientation(e->first_vertex->point, e->second_vertex->point, v->point) != cg::CG_RIGHT);
        }
        return result;
    }
}
