 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include "intersect.hpp"
#include "tracing.hpp"

#include <limits>

namespace geometry
{

PolynomialEquation::PolynomialEquation(double x0, double x1, double x2)
    : polynome_({x0, x1, x2})
{
}

void PolynomialEquation::solve()
{
    switch (polynome_.size())
    {
    case 3u:
        solveOrder2();
        return;
    default:
        TRACE("Cannot solve %d order polynomial equation", polynome_.size() - 1);
        // throw std::runtime_error("Cannot solve that polynomial equation");
        return;
    }
}

void PolynomialEquation::solveOrder2()
{
    const double &x0 = polynome_[0];
    const double &x1 = polynome_[1];
    const double &x2 = polynome_[2];

    const double delta = x1 * x1 - 4 * x2 * x0;
    if (delta < 0.0) {
        return;
    }
    const double delta_sqrt = std::sqrt(delta);

    const double root_1 = (-x1 - delta_sqrt) / (2 * x2);
    const double root_2 = (-x1 + delta_sqrt) / (2 * x2);

    roots_ = std::move(std::vector<double>({root_1, root_2}));
}

boost::optional<double> PolynomialEquation::getMinNonNegative() const
{
    if (!roots_.is_initialized() || roots_->size() == 0u) {
        return {};
    }

    bool min_found = false;
    double min_value = std::numeric_limits<double>::max();
    for (const double &value : *roots_) {
        if (value < 0.0) {
            continue;
        }

        min_found = true;
        min_value = std::min(min_value, value);
    }
    if (min_found) {
        return min_value;
    }
    return {};
}

boost::optional<Vector2D> KinematicIntersect::intersect(
    const Vector2D &actor_position, const double max_actor_speed,
    const Vector2D &target_position, const Vector2D &target_velocity)
{
    // FYI: x_ means that x is a vector not a scalar
    // d_ + v_ * t = p_ * t
    // Left: vector from actor position to target_position + target velocity times time
    // Right: interception velocity times time
    // Unknowns: interception velocity and time (time required for actor to travel), 3 unknowns with one 2 dimensional vector equation
    // p_ = direction_ * max_actor_speed * t
    // d_ + v_ * t = direction_ * max_actor_speed * t
    // direction_ = [cos(alpha); sin(alpha)]
    // Unknowns: alpha and time, 2 unknowns with 2 non linear scalar equations
    // dx + vx * t = cos(alpha) * s * t
    // dy + vy * t = sin(alpha) * s * t
    // square and sum
    // dx^2 + dy^2 + 2 * (dx * vx + dy * vy) * t  + (vx^2 + vy^2) * t^2 = s^2 * t^2
    // (vx^2 + vy^2 - s^2) * t^2 + 2 * (dx * vx + dy * vy) * t + (dx^2 + dy^2) = 0
    // quadratic equation in respect to time
    // a = (vx^2 + vy^2 - s^2)
    // b = 2 * (dx * vx + dy * vy)
    // c = dx^2 + dy^2
    // a * t^2 + b * t + c = 0
    // delta = b^2 - 4 * a * c
    // t1 = (-b - sqrt(delta)) / (2 * a)
    // t2 = (-b + sqrt(delta)) / (2 * a)

    const Vector2D d_ = target_position - actor_position;
    const double dx = d_.x;
    const double dy = d_.y;
    const double vx = target_velocity.x;
    const double vy = target_velocity.y;
    const double s = max_actor_speed;

    const double a = vx * vx + vy * vy - s * s;
    const double b = 2 * (dx * vx + dy * vy);
    const double c = dx * dx + dy * dy;

    PolynomialEquation equation(c, b, a);
    equation.solve();
    boost::optional<double> time = equation.getMinNonNegative();
    if (time.is_initialized()) {
        return target_position + target_velocity * (*time);
    }

    TRACE("Intersection point not found.");
    return {};
}

} // namespace geometry
