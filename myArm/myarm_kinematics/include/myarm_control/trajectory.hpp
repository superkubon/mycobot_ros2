#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace myarm_control
{

class QuinticTrajectory
{
public:
    using Vector6 = Eigen::Matrix<double, 6, 1>;

    QuinticTrajectory() = default;

    QuinticTrajectory(
        const Vector6& q0,
        const Vector6& q1,
        double duration)
    {
        reset(q0, q1, duration);
    }

    void reset(
        const Vector6& q0,
        const Vector6& q1,
        double duration)
    {
        q0_ = q0;
        dq_ = q1 - q0;
        duration_ = std::max(duration, 0.01);
        active_ = true;
    }

    bool active() const
    {
        return active_;
    }

    double duration() const
    {
        return duration_;
    }

    void stop()
    {
        active_ = false;
    }

    Vector6 position(double t) const
    {
        double s = 0.0;
        double ds = 0.0;
        evaluate(t, s, ds);

        (void)ds;
        return q0_ + s * dq_;
    }

    Vector6 velocity(double t) const
    {
        double s = 0.0;
        double ds = 0.0;
        evaluate(t, s, ds);

        (void)s;
        return dq_ * ds;
    }

    Vector6 acceleration(double t) const
    {
        if (duration_ <= 0.0)
            return Vector6::Zero();

        const double tau =
            std::clamp(t / duration_, 0.0, 1.0);

        const double d2s_dt2 =
            (60.0 * tau - 180.0 * tau * tau +
             120.0 * tau * tau * tau) /
            (duration_ * duration_);

        return dq_ * d2s_dt2;
    }

    bool finished(double t) const
    {
        return t >= duration_;
    }

private:
    void evaluate(
        double t,
        double& s,
        double& ds) const
    {
        if (t <= 0.0)
        {
            s = 0.0;
            ds = 0.0;
            return;
        }

        if (t >= duration_)
        {
            s = 1.0;
            ds = 0.0;
            return;
        }

        const double tau =
            t / duration_;

        s =
            10.0 * tau * tau * tau -
            15.0 * tau * tau * tau * tau +
             6.0 * tau * tau * tau * tau * tau;

        ds =
            (30.0 * tau * tau -
             60.0 * tau * tau * tau +
             30.0 * tau * tau * tau * tau) /
            duration_;
    }

    Vector6 q0_{Vector6::Zero()};
    Vector6 dq_{Vector6::Zero()};
    double duration_{0.5};
    bool active_{false};
};

} // namespace myarm_control
