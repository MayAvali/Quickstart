package org.firstinspires.ftc.teamcode.team.subsystems;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.team.libraries.PIDFController;
import org.firstinspires.ftc.teamcode.team.libraries.Conversion;
import org.firstinspires.ftc.teamcode.team.opmodes.teleop.testing.LauncherTestTele;

public class turret {

    private final Servo turretDriverL;
    private final Servo turretDriverR;
    private final DcMotorEx launcherMotorFront;
    private final DcMotorEx launcherMotorRear;

    //------------------------------
    public static final double fallbackVelocity = Conversion.RPMtoVel(1500);
    public static final double MotionRangeDegrees = 210;

    public static final double CenterOffsetDegrees = 2;
    //------------------------------
    public static class launcherPIDF {
        public static double P = 0.012;
        public static double I = 0.00001;
        public static double D = 0.02;
        public static double F = 0.00046;
    }
    public static PIDFController LauncherPIDF = new PIDFController(launcherPIDF.P, launcherPIDF.I, launcherPIDF.D, launcherPIDF.F);
    public turret(Servo turretDriverL, Servo turretDriverR, DcMotorEx launcherMotorFront, DcMotorEx launcherMotorRear) {
        this.turretDriverL = turretDriverL;
        this.turretDriverR = turretDriverR;

        this.launcherMotorFront = launcherMotorFront;
        this.launcherMotorRear = launcherMotorRear;
        launcherMotorFront.setDirection(DcMotorEx.Direction.REVERSE);
        launcherMotorRear.setDirection(DcMotorEx.Direction.REVERSE);

        LauncherPIDF.reset();
        LauncherPIDF.integralRange = 30;
    }

    public void launcherSetVel(double targetVelocity){
        double power = LauncherPIDF.update(targetVelocity,launcherMotorRear.getVelocity());
        launcherMotorFront.setPower(power);
        launcherMotorRear.setPower(power);
    }

    public void launcherUpdate () {
        launcherSetVel(LauncherPIDF.getLastSetPoint());
    }

    public void turretSetTarget (double Angle){
        Angle = Math.max(-MotionRangeDegrees/2, Math.min(Angle,MotionRangeDegrees/2));
        double ServoTarget = 0.5 - ((Angle+CenterOffsetDegrees)/(MotionRangeDegrees));
        ServoTarget = Math.max(0.0, Math.min(1.0, ServoTarget));
        turretDriverL.setPosition(ServoTarget);
        turretDriverR.setPosition(ServoTarget);
    }

    public static double launcherDistToVelocity(double Distance) {
                Distance = Math.max(20, Math.min(90, Distance));
                return 0.35*((0.00016752*Math.pow(Distance,4))-(0.0408935*Math.pow(Distance,3))+(3.67967*Math.pow(Distance,2))-(126.85945*Distance)+3380.61105);
        }


    public double launcherGetVel() {
        return LauncherPIDF.getlastvalue();
    }
    public double launcherGetP() {
        return LauncherPIDF.getP();
    }
    public double launcherGetI() {
        return LauncherPIDF.getI();
    }
    public double launcherGetD() {
        return LauncherPIDF.getD();
    }
    public double launcherGetF() {
        return LauncherPIDF.getF();
    }

    public static double launcherGetTarget(){ return LauncherPIDF.getLastSetPoint();}

}
