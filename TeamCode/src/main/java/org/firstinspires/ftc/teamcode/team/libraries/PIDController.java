package org.firstinspires.ftc.teamcode.team.libraries;

public class PIDController extends PIDFController {

    public PIDController(double proportional, double integral, double derivative) {
        super(proportional, integral, derivative, 0);
    }

    public PIDController(
            double proportional,
            double integral,
            double derivative,
            double integralCap,
            double decayFactor,
            double integralRange) {
        super(proportional, integral, derivative, 0, integralCap, decayFactor, integralRange);
    }

    public PIDController(
            double proportional,
            double integral,
            double derivative,
            double feedforward,
            double integralCap,
            double decayFactor,
            double integralRange) {
        super(proportional, integral, derivative, feedforward, integralCap, decayFactor, integralRange);
    }
}
