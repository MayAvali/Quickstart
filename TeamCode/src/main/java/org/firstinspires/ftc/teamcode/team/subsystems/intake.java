package org.firstinspires.ftc.teamcode.team.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class intake {

    private final DcMotorEx intake1;
    private final DcMotorEx intake2;
    public intake(DcMotorEx intake1, DcMotorEx intake2) {
        this.intake1 = intake1;
        this.intake2 = intake2;
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power){
        intake1.setPower(power);
        intake2.setPower(power);
    }

    public void runIntakeDumb(double reverse, double forward){
        intake1.setPower(forward-reverse);
        intake2.setPower(forward-reverse);
    }

}
