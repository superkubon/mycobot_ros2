#pragma once

#include <algorithm>
#include <cmath>

namespace myarm_control
{

class PID
{
public:
    struct Config
    {
        double kp{0.8};
        double ki{0.0};
        double kd{0.03};

        // Output is a position correction in radians.
        double output_min{-0.05};
        double output_max{0.05};

        // Integral state is in rad*s.
        double integral_min{-0.20};
        double integral_max{0.20};

        // Small errors are ignored to avoid chatter around zero.
        double deadband{0.0005};
    };

    PID()
        : cfg_(Config{})
    {
    }

    explicit PID(const Config& cfg)
        : cfg_(cfg)
    {
    }

    void reset()
    {
        integral_ = 0.0;
        last_output_ = 0.0;
    }

    double update(
        double error,
        double measured_velocity,
        double dt)
    {
        if (dt <= 0.0 || !std::isfinite(dt))
            return last_output_;

        if (std::abs(error) <= cfg_.deadband)
            error = 0.0;

        // Candidate integral.
        const double candidate_integral =
            std::clamp(
                integral_ + error * dt,
                cfg_.integral_min,
                cfg_.integral_max);

        // Derivative-on-measurement avoids a kick when q_ref changes.
        const double unsaturated =
            cfg_.kp * error +
            cfg_.ki * candidate_integral -
            cfg_.kd * measured_velocity;

        const double saturated =
            std::clamp(
                unsaturated,
                cfg_.output_min,
                cfg_.output_max);

        // Conditional anti-windup:
        // only accept integral growth when it does not push farther
        // into an already-active saturation limit.
        const bool saturated_high =
            unsaturated > cfg_.output_max &&
            error > 0.0;

        const bool saturated_low =
            unsaturated < cfg_.output_min &&
            error < 0.0;

        if (!saturated_high && !saturated_low)
            integral_ = candidate_integral;

        last_output_ = saturated;
        return saturated;
    }

    double integral() const { return integral_; }

private:
    Config cfg_;
    double integral_{0.0};
    double last_output_{0.0};
};

} // namespace myarm_control
