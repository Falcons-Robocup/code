// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef GEOMETRY_INTERSECT_HPP
#define GEOMETRY_INTERSECT_HPP

#include "vector2d.hpp"

#include <vector>
#include <boost/optional.hpp>

namespace geometry
{

class PolynomialEquation
{
public:
    PolynomialEquation(double x0, double x1, double x2);

    void solve();

    boost::optional<double> getMinNonNegative() const;

private:
    std::vector<double> polynome_;
    boost::optional<std::vector<double>> roots_;

    void solveOrder2();
};

class KinematicIntersect
{
public:
    static boost::optional<Vector2D> intersect(
        const Vector2D &actor_position, double max_actor_speed,
        const Vector2D &target_position, const Vector2D &target_velocity);
};

} // namespace geometry

#endif // GEOMETRY_INTERSECT_HPP
