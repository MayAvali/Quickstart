package org.firstinspires.ftc.teamcode.team.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.team.libraries.GamepadButton;
import org.firstinspires.ftc.teamcode.team.subsystems.MecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpDrivetrain", group = "Linear OpMode")
public class DrivetrainTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        if (isStopRequested()) return;

        GamepadButton launchButton = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.RIGHT_BUMPER);

        MecanumDrive drivetrain = new MecanumDrive(
                (DcMotorEx) hardwareMap.dcMotor.get("leftFront"),
                (DcMotorEx) hardwareMap.dcMotor.get("leftBack"),
                (DcMotorEx) hardwareMap.dcMotor.get("rightFront"),
                (DcMotorEx) hardwareMap.dcMotor.get("rightBack"),
                hardwareMap.get(IMU.class, "imu")
        );

        while (opModeIsActive())
        {
            drivetrain.botOrientedDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_trigger);

            telemetry.addData("RightBumper", gamepad1.right_bumper);

            telemetry.addData("Front Left Motor Power " , drivetrain.getFrontLeftPower());
            telemetry.addData("Back Left Motor Power " , drivetrain.getBackLeftPower());
            telemetry.addData("Front Right Motor Power " , drivetrain.getFrontRightPower());
            telemetry.addData("Back Right Motor Power " , drivetrain.getBackRightPower());

            telemetry.addData("Left Stick X " , gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y " , gamepad1.left_stick_y);
            telemetry.addData("Right Stick X " , gamepad1.right_stick_x);
            telemetry.addData("Right Trigger " , gamepad1.right_trigger);
            telemetry.update();
        }
    }
}
