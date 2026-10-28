package org.firstinspires.ftc.teamcode.team.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {

    private final DcMotorEx intake1;
    private final DcMotorEx intake2;
    public Intake(DcMotorEx intake1, DcMotorEx intake2) {
        this.intake1 = intake1;
        this.intake2 = intake2;
    }

}
