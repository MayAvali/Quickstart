package org.firstinspires.ftc.teamcode.team.opmodes.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.team.libraries.GamepadButton;
import org.firstinspires.ftc.teamcode.team.subsystems.gate;
import org.firstinspires.ftc.teamcode.team.subsystems.intake;
import org.firstinspires.ftc.teamcode.team.subsystems.mecanumDrive;
import org.firstinspires.ftc.teamcode.team.subsystems.turret;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "CompetitionTele", group = "Linear OpMode")
public class CompetitionTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        GamepadButton launchButton = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.RIGHT_BUMPER);

        mecanumDrive Drivetrain = new mecanumDrive(
                (DcMotorEx) hardwareMap.dcMotor.get("leftFront"),
                (DcMotorEx) hardwareMap.dcMotor.get("leftBack"),
                (DcMotorEx) hardwareMap.dcMotor.get("rightFront"),
                (DcMotorEx) hardwareMap.dcMotor.get("rightBack"),
                hardwareMap.get(IMU.class, "imu")
        );

        intake Intake = new intake(
                (DcMotorEx) hardwareMap.dcMotor.get("intakeL"),
                (DcMotorEx) hardwareMap.dcMotor.get("intakeR")
        );

        gate ServoGate = new gate(
                hardwareMap.servo.get("gate")
        );

        turret Turret = new turret(
                hardwareMap.servo.get("turretL"),
                hardwareMap.servo.get("turretR"),
                (DcMotorEx) hardwareMap.dcMotor.get("launcherFront"),
                (DcMotorEx) hardwareMap.dcMotor.get("launcherRear")
        );

        waitForStart();
        if (isStopRequested()) return;

        ServoGate.forceCloseGate();

        while (opModeIsActive())
        {
            Drivetrain.botOrientedDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_trigger);

            telemetry.addData("RightBumper", gamepad1.right_bumper);

            telemetry.addData("Front Left Motor Power " , Drivetrain.getFrontLeftPower());
            telemetry.addData("Back Left Motor Power " , Drivetrain.getBackLeftPower());
            telemetry.addData("Front Right Motor Power " , Drivetrain.getFrontRightPower());
            telemetry.addData("Back Right Motor Power " , Drivetrain.getBackRightPower());

            telemetry.addData("Left Stick X " , gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y " , gamepad1.left_stick_y);
            telemetry.addData("Right Stick X " , gamepad1.right_stick_x);
            telemetry.addData("Right Trigger " , gamepad1.right_trigger);
            telemetry.update();
        }
    }
}

