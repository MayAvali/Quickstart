package org.firstinspires.ftc.teamcode.team.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.team.libraries.PIDController;
import org.firstinspires.ftc.teamcode.team.libraries.PIDFController;

public class intake {

    private final DcMotorEx intake1;
    private final DcMotorEx intake2;
    private double lastTargetVelocity = 0;

    public static class intakePIDF {
        public static double P = 0.00035;
        public static double I = 0.0000001;
        public static double D = 0.00001;
        public static double F = 0.00041;
    }
    public static PIDFController IntakePIDF = new PIDFController(intake.intakePIDF.P, intake.intakePIDF.I,intake.intakePIDF.D, intake.intakePIDF.F);
    public intake(DcMotorEx intake1, DcMotorEx intake2) {
        this.intake1 = intake1;
        this.intake2 = intake2;
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        IntakePIDF.reset();
        IntakePIDF.withIntegralRange(30);
    }

    public void setPower(double power){
        intake1.setPower(power);
        intake2.setPower(power);
    }

    public void runIntakeDumb(double reverse, double forward){
        intake1.setPower(forward-reverse);
        intake2.setPower(forward-reverse);
    }

    public void runIntake(double reverse, double forward){
//        reverse = Math.min(1, reverse);
//        forward = Math.min(1, forward);
        lastTargetVelocity = 2000 * (1*(1-forward) - 2*reverse);
        double power = IntakePIDF.update(lastTargetVelocity, intake1.getVelocity());

        intake1.setPower(power);
        intake2.setPower(power);
    }

    public double intakeGetTarget() {
        return lastTargetVelocity;
    }
    public double intakeGetVel() {
        return IntakePIDF.getlastvalue();
    }
    public double intakeGetP() {
        return IntakePIDF.getP();
    }
    public double intakeGetI() {
        return IntakePIDF.getI();
    }
    public double intakeGetD() {
        return IntakePIDF.getD();
    }
    public double intakeGetF() {
        return IntakePIDF.getF();
    }

}
