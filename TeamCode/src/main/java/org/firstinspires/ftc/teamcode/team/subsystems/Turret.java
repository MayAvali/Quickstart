package org.firstinspires.ftc.teamcode.team.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.team.libraries.PIDController;
import org.firstinspires.ftc.teamcode.team.libraries.Conversion;

public class Turret {

    private final Servo turretDriverL;
    private final Servo turretDriverR;
    private final DcMotorEx launcherMotorFront;
    private final DcMotorEx launcherMotorRear;

    //------------------------------
    private final double fallbackVelocity = Conversion.RPMtoVel(3200);
    public static final double MotionRangeDegrees = 248.88888889;

    //------------------------------
    public static class launcherPID {
        public static double P = 0;
        public static double I = 0;
        public static double D = 0;
    }
    public static PIDController LauncherPID = new PIDController(launcherPID.P, launcherPID.I, launcherPID.D);
    public Turret(Servo turretDriverL, Servo turretDriverR, DcMotorEx launcherMotorFront, DcMotorEx launcherMotorRear) {
        this.turretDriverL = turretDriverL;
        this.turretDriverR = turretDriverR;

        this.launcherMotorFront = launcherMotorFront;
        this.launcherMotorRear = launcherMotorRear;
//        launcherMotorFront.setDirection(DcMotorEx.Direction.REVERSE);
//        launcherMotorRear.setDirection(DcMotorEx.Direction.REVERSE);

        LauncherPID.reset();
        LauncherPID.withIntegralRange(50);
    }
    public void launcherSetVel(double targetVelocity){
        double power = LauncherPID.update(targetVelocity,launcherMotorFront.getVelocity());
        launcherMotorFront.setPower(power);
        launcherMotorRear.setPower(power);
    }

}
