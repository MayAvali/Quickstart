package org.firstinspires.ftc.teamcode.team.opmodes.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.team.libraries.GamepadButton;
import org.firstinspires.ftc.teamcode.team.subsystems.Gate;
import org.firstinspires.ftc.teamcode.team.subsystems.Intake;
import org.firstinspires.ftc.teamcode.team.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.team.subsystems.Turret;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "CompetitionTele", group = "Linear OpMode")
public class CompetitionTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        GamepadButton launchButton = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.RIGHT_BUMPER);

        MecanumDrive drivetrain = new MecanumDrive(
                (DcMotorEx) hardwareMap.dcMotor.get("leftFront"),
                (DcMotorEx) hardwareMap.dcMotor.get("leftBack"),
                (DcMotorEx) hardwareMap.dcMotor.get("rightFront"),
                (DcMotorEx) hardwareMap.dcMotor.get("rightBack"),
                hardwareMap.get(IMU.class, "imu")
        );

        Intake intake = new Intake(
                (DcMotorEx) hardwareMap.dcMotor.get("intakeL"),
                (DcMotorEx) hardwareMap.dcMotor.get("intakeR")
        );

        Gate servoGate = new Gate(
                hardwareMap.servo.get("gate")
        );

        Turret turret = new Turret(
                hardwareMap.servo.get("turretL"),
                hardwareMap.servo.get("turretR"),
                (DcMotorEx) hardwareMap.dcMotor.get("launcherFront"),
                (DcMotorEx) hardwareMap.dcMotor.get("launcherRear")
        );

        waitForStart();
        if (isStopRequested()) return;

        servoGate.forceCloseGate();

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

