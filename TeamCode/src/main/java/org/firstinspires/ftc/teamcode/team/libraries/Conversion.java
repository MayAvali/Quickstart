package org.firstinspires.ftc.teamcode.team.libraries;

public class Conversion {
    static final int TicksPerRev = 28;
    public static double VeltoRPM(double RPM) {
        return (RPM / TicksPerRev) * 60;
    }
    public static double RPMtoVel(double RPM) {
        return (RPM * TicksPerRev) / 60;
    }
}
