package org.firstinspires.ftc.teamcode.team.libraries;

public class PIDFController {
    // would be final, but dashboard
    public double proportional;
    public double integral;
    public double derivative;
    public double feedforward;
    public double integralCap;
    // what to multiply by every second
    public double decayFactor;
    // range before integral kicks in
    public double integralRange;

    private boolean firstLoop = true;
    private long lastTime = 0;
    private double lastError = 0;
    private double errorAccum = 0;
    private boolean lastOutOfRange = true;

    private double lastP = 0;
    private double lastI = 0;
    private double lastD = 0;
    private double lastF = 0;
    private double lastvalue = 0;

    private double lastSetPoint = 0;

    public PIDFController(double proportional, double integral, double derivative, double feedforward) {
        this(
                proportional,
                integral,
                derivative,
                feedforward,
                Double.POSITIVE_INFINITY,
                1,
                Double.POSITIVE_INFINITY
        );
    }

    public PIDFController(
            double proportional,
            double integral,
            double derivative,
            double feedforward,
            double integralCap,
            double decayFactor,
            double integralRange) {
        this.proportional = proportional;
        this.integral = integral;
        this.derivative = derivative;
        this.feedforward = feedforward;
        this.integralCap = integralCap;
        this.decayFactor = decayFactor;
        this.integralRange = integralRange;
    }

    public PIDFController withFeedforward(double f) {
        return new PIDFController(proportional, integral, derivative, f, integralCap, decayFactor, integralRange);
    }

    public PIDFController withIntegralCap(double cap) {
        return new PIDFController(proportional, integral, derivative, feedforward, cap, decayFactor, integralRange);
    }

    public PIDFController withDecayFactor(double decay) {
        return new PIDFController(proportional, integral, derivative, feedforward, integralCap, decay, integralRange);
    }

    public PIDFController withIntegralRange(double range) {
        return new PIDFController(proportional, integral, derivative, feedforward, integralCap, decayFactor, range);
    }

    public double update(double setpoint, double value) {
        lastSetPoint = setpoint;
        long time = System.currentTimeMillis();
        double error = setpoint - value;
        if (firstLoop) {
            lastTime = time;
            lastError = error;
            firstLoop = false;
        }
        long delta = time - lastTime;

        double i = 0;
        if (Math.abs(error) < integralRange) {
            if (lastOutOfRange) {
                lastOutOfRange = false;
                errorAccum = 0;
            }
            errorAccum += (error + lastError) / 2 * delta;
            errorAccum = Math.max(-integralCap, Math.min(integralCap, errorAccum));
            errorAccum *= Math.pow(decayFactor, (double) delta / 1000);
            i = integral * errorAccum;
        } else {
            lastOutOfRange = true;
        }

        double p = proportional * error;

        // delta can be 0 on back-to-back loops; guard so we don't divide by zero
        double d = delta > 0 ? derivative * (error - lastError) / delta : 0;

        // feedforward acts on the setpoint, not the error
        double f = feedforward * setpoint;

        lastTime = time;
        lastError = error;

        lastP = p;
        lastI = i;
        lastD = d;
        lastF = f;
        lastvalue = value;

        return p + i + d + f;
    }

    public void reset() {
        firstLoop = true;
        lastTime = 0;
        lastError = 0;
        errorAccum = 0;
        lastOutOfRange = true;
    }

    public double getErrorAccum() { return errorAccum; }
    public double getP() { return lastP; }
    public double getI() { return lastI; }
    public double getD() { return lastD; }
    public double getF() { return lastF; }
    public double getlastvalue() { return lastvalue; }
    public double getLastSetPoint() {return lastSetPoint;}
}
