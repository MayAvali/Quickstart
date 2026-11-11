package org.firstinspires.ftc.teamcode.team.opmodes.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import com.pedropathing.math.Pose;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.team.libraries.Conversion;
import org.firstinspires.ftc.teamcode.team.libraries.GamepadButton;
import org.firstinspires.ftc.teamcode.team.libraries.LocalizationUtils;
import org.firstinspires.ftc.teamcode.team.libraries.PositionBank;
import org.firstinspires.ftc.teamcode.team.subsystems.gate;
import org.firstinspires.ftc.teamcode.team.subsystems.intake;
import org.firstinspires.ftc.teamcode.team.subsystems.mecanumDrive;
import org.firstinspires.ftc.teamcode.team.subsystems.turret;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "CompetitionTele", group = "Linear OpMode")
public class CompetitionTele extends LinearOpMode {
    private Follower follower;
    public GoBildaPinpointDriver pinpoint;
    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

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
                hardwareMap.servo.get("gateL"),
                hardwareMap.servo.get("gateR")
        );

        turret Turret = new turret(
                hardwareMap.servo.get("turretL"),
                hardwareMap.servo.get("turretR"),
                (DcMotorEx) hardwareMap.dcMotor.get("launcherFront"),
                (DcMotorEx) hardwareMap.dcMotor.get("launcherRear")
        );

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        follower = Constants.create(hardwareMap);
        Object savedPosition = blackboard.get("BotPose");
        Pose startPosition = (savedPosition instanceof Pose) ? (Pose) savedPosition : PositionBank.HiveInitPosition;
        follower.setPose(startPosition);

        GamepadButton StateSwitchButton = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.RIGHT_BUMPER);
        GamepadButton GoalSwitchButton = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.LEFT_BUMPER);
        GamepadButton OdoResetButton = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.SHARE);
        GamepadButton TargetResetButton = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.START);

        GamepadButton ManualSpeedToggle = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.triangle);
        GamepadButton StaticTurretToggle = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.circle);

        GamepadButton TargetForwardShiftButton = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.DPAD_UP);
        GamepadButton TargetRightShiftButton = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.DPAD_RIGHT);
        GamepadButton TargetDownShiftButton = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.DPAD_DOWN);
        GamepadButton TargetLeftShiftButton = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.DPAD_LEFT);

        Pose TargetShift = new Pose(0,0,0);

        Pose leftCellTargetPosition = PositionBank.LeftCellTargetPosition;
        Pose rightCellTargetPosition = PositionBank.RightCellTargetPosition;

        boolean fixedTurret = false;
        boolean manualSpeed = false;
        double fallbackTargetVelocity = Conversion.RPMtoVel(1500);

        boolean targetResetIdle = false;

        boolean launchActive = false;
        boolean rightCellUp = false;

        waitForStart();
        if (isStopRequested()) return;

        ServoGate.forceCloseGate();

        while (opModeIsActive())
        {
            if(ManualSpeedToggle.isPressed()){
                manualSpeed = !manualSpeed;
            }
            if(StaticTurretToggle.isPressed()){
                fixedTurret = !fixedTurret;
            }
            if(StateSwitchButton.isPressed()){
                launchActive = !launchActive;
            }
            if(GoalSwitchButton.isPressed()){
                rightCellUp = !rightCellUp;
            }
            if(TargetResetButton.isPressed()){
                TargetShift = new Pose(0,0,0);
            }
            if(OdoResetButton.isPressed() && !TargetResetButton.isDown()){
                follower.setPose(PositionBank.FallbackResetPosition);
            }
            if(OdoResetButton.isPressed() && TargetResetButton.isDown()){
                if (!targetResetIdle) {
                    targetResetIdle = true;
                } else {
                    targetResetIdle = false;
                    pinpoint.recalibrateIMU();
                    sleep(400);
                    follower.setPose(PositionBank.FallbackResetPosition);
                }
            }

            if(TargetForwardShiftButton.isPressed() && !fixedTurret){
                TargetShift = TargetShift.plus(new Pose(1,0,0));
            }
            if(TargetForwardShiftButton.isPressed() && fixedTurret){
                fallbackTargetVelocity += 25;
            }
            if(TargetRightShiftButton.isPressed() && !fixedTurret){
                TargetShift = TargetShift.plus(new Pose(0,-1,0));
            }
            if(TargetDownShiftButton.isPressed() && !fixedTurret){
                TargetShift = TargetShift.plus(new Pose(1,0,0));
            }
             if(TargetDownShiftButton.isPressed() && fixedTurret){
                fallbackTargetVelocity -= 25;
            }
            if(TargetLeftShiftButton.isPressed() && !fixedTurret){
                TargetShift = TargetShift.plus(new Pose(0,1,0));
            }
            rightCellTargetPosition = rightCellTargetPosition.plus(TargetShift);
            leftCellTargetPosition = leftCellTargetPosition.plus(TargetShift);

            if (launchActive) ServoGate.openGate(); else ServoGate.closeGate();

            if(launchActive) {
                if (rightCellUp) {
                    if(!fixedTurret){

                    }else{

                    }
                } else {
                    if(!fixedTurret){

                    }else{

                    }
                }
            } else {
                if (rightCellUp) {
                    if(!fixedTurret){

                    }else{

                    }
                } else {
                    if(!fixedTurret){

                    }else{

                    }
                }
            }
            if (!manualSpeed){
                if (!targetResetIdle) {
                    if (rightCellUp){
                        Turret.launcherSetVel(turret.launcherDistToVelocity(LocalizationUtils.getDistancePose(follower.pose(), PositionBank.RightCellTargetPosition)));
                    }else{
                        Turret.launcherSetVel(turret.launcherDistToVelocity(LocalizationUtils.getDistancePose(follower.pose(), PositionBank.LeftCellTargetPosition)));
                    }
                } else {
                    Turret.launcherSetVel(fallbackTargetVelocity);
                }
            }

            Drivetrain.botOrientedDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_trigger);

            Intake.runIntakeDumb(gamepad1.left_trigger, gamepad1.right_trigger);



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

