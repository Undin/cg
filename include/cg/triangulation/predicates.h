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
            result += ((Scalar) v[i]->getPoint().x - d->getPoint().x) *
                      ((Scalar) v[next(i)]->getPoint().y - d->getPoint().y) *
                      (pointSqr<Scalar>(v[prev(i)]->getPoint()) - pointSqr<Scalar>(d->getPoint())) -
                      ((Scalar) v[i]->getPoint().x - d->getPoint().x) *
                      ((Scalar) v[prev(i)]->getPoint().y - d->getPoint().y) *
                      (pointSqr<Scalar>(v[next(i)]->getPoint()) - pointSqr<Scalar>(d->getPoint()));
        }
        return result;
    }

    struct inCircleD
    {
        boost::optional<CircleContent> operator()(const std::array<Vertex, 3> &v, const Vertex &d) const
        {
            if (d->isInfinity())
            {
                return CircleContent::OUT_CIRCLE;
            }
            bool infFace = false;
            for (int i = 0; i < 3; i++)
            {
                if (v[i]->isInfinity())
                {
                    infFace = true;
                    if (orientation(v[next(i)]->getPoint(), v[prev(i)]->getPoint(), d->getPoint()) == cg::CG_LEFT)
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
                resultAbs += fabs((v[i]->getPoint().x - d->getPoint().x) *
                                  (v[next(i)]->getPoint().y - d->getPoint().y) *
                                  (pointSqr<double>(v[prev(i)]->getPoint()) - pointSqr<double>(d->getPoint()))) +
                             fabs((v[i]->getPoint().x - d->getPoint().x) *
                                  (v[prev(i)]->getPoint().y - d->getPoint().y) *
                                  (pointSqr<double>(v[next(i)]->getPoint()) - pointSqr<double>(d->getPoint())));
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

    bool badEdge(const Edge &e1)
    {
        Vertex a1 = e1->getFirstVertex();
        Vertex b1 = e1->getSecondVertex();
        Vertex c1 = e1->getNextEdge()->getSecondVertex();
        Edge e2 = e1->getTwin();
        Vertex a2 = e2->getFirstVertex();
        Vertex b2 = e2->getSecondVertex();
        Vertex c2 = e2->getNextEdge()->getSecondVertex();
        return inCircle(a1, b1, c1, c2) == CircleContent::IN_CIRCLE || inCircle(a2, b2, c2, c1) == CircleContent::IN_CIRCLE;
    }

    bool inFace(const Face &f, const Vertex &v)
    {
        bool result = true;
        Edge e = f->getEdge();
        for (int i = 0; i < 3 && result; i++, e = e->getNextEdge())
        {
            result &= (e->getFirstVertex()->isInfinity()  ||
                       e->getSecondVertex()->isInfinity() ||
                       orientation(e->getFirstVertex()->getPoint(), e->getSecondVertex()->getPoint(), v->getPoint()) != cg::CG_RIGHT);
        }
        return result;
    }

    bool inFace(const Face &f, const point_2 &p)
    {
        return inFace(f, Vertex(new VertexHandle(p)));
    }
}
