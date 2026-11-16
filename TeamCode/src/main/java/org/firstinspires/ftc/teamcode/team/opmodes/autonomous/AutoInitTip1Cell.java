package org.firstinspires.ftc.teamcode.team.opmodes.autonomous;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.team.subsystems.gate;
import org.firstinspires.ftc.teamcode.team.subsystems.intake;
import org.firstinspires.ftc.teamcode.team.subsystems.turret;

import static com.pedropathing.ivy.commands.Commands.*;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "1. Autonomous Tip 1 Cell", group = "Autonomous")
public class AutoInitTip1Cell extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.2669, 7.0872, 90);
    private final Pose shootPreload = poseFactory.of(58, 19, 90);
    private final Pose getGardenStart = poseFactory.of(58, 30, 90);
    private final Pose getGarden = poseFactory.of(10.5, 9.5, -90.001);
    private final Pose getGardenControl1 = poseFactory.of(29.493, 29.1373, 0);
    private final Pose shootGarden = poseFactory.of(58, 19, 90);
    private final Pose shootGardenSegment1Heading = poseFactory.of(58, 24, -90);
    private final Pose shootGardenSegment2Start = poseFactory.of(58, 24, -90);
    private final Pose shootGardenSegment2End = poseFactory.of(58, 24, 90);
    private final Pose preparePushStart = poseFactory.of(58, 24, 90);
    private final Pose preparePush = poseFactory.of(9, 62.7412, 0);
    private final Pose pushPartner = poseFactory.of(9, 99, 0);

    private intake Intake;
    private gate ServoGate;
    private turret Turret;

    // --- Subsystem Updating Functions (allows PID to function in the opmode loop) ---
    public void updateIntake() {
        if (Intake != null) {
            Intake.intakeUpdate();
        }
    }

    public void updateLauncher() {
        if (Turret != null) {
            Turret.launcherUpdate();
        }
    }

    // --- Ivy Control Flow Commands ---

    /** Non-blocking waiting function */
    public Command waitSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        return sequential(
                instant(timer::reset),
                waitUntil(() -> timer.seconds() >= seconds)
        );
    }

    /** Intake speed setting function (reverse, forward power inputs) */
    public Command setIntakeSpeed(double reverse, double forward) {
        return instant(() -> Intake.runIntake(reverse, forward));
    }

    /** Intake velocity setting function (direct target velocity input) */
    public Command setIntakeVelocity(double targetVelocity) {
        return instant(() -> Intake.setTargetVelocity(targetVelocity));
    }

    /** Reverses the intake for a specified duration in seconds, then stops it */
    public Command reverseIntake(double seconds) {
        return sequential(
                setIntakeSpeed(1, 0),
                waitSeconds(seconds),
                setIntakeSpeed(0, 0)
        );
    }

    /** Reverses the intake for 0.2 seconds, then stops it */
    public Command reverseIntake() {
        return reverseIntake(0.2);
    }

    /** Servo gate opening function */
    public Command openGate() {
        return instant(() -> ServoGate.openGate());
    }

    /** Servo gate closing function */
    public Command closeGate() {
        return instant(() -> ServoGate.closeGate());
    }

    /** Turret target angle function */
    public Command setTurretAngle(double angleDegrees) {
        return instant(() -> Turret.turretSetTarget(angleDegrees));
    }

    /** Launcher target speed setting function */
    public Command setLauncherSpeed(double targetVelocity) {
        return instant(() -> Turret.launcherSetVel(targetVelocity));
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                setTurretAngle(0),
                closeGate(),
                setLauncherSpeed(775),
                setIntakeSpeed(0,0),
                waitSeconds(1.5),

                follow(follower, shootPreload()),

                //setIntakeSpeed(0,0.5),
                openGate(),
                waitSeconds(2),
                reverseIntake(),
                waitSeconds(1),
                closeGate(),
                //setIntakeSpeed(0,0),

                follow(follower, getGarden()),

                waitSeconds(2),

                follow(follower, shootGarden()),

                openGate(),
                waitSeconds(2),
                reverseIntake(),
                waitSeconds(1),
                closeGate(),

                follow(follower, preparePush()),

                setIntakeSpeed(0,1),

                follow(follower, pushPartner()),

                waitSeconds(0.5),
                setTurretAngle(-60),
                waitSeconds(0.5),
                setTurretAngle(60),
                waitSeconds(1),
                setTurretAngle(0)
        );
    }

    @Override
    public void runOpMode() {

        Intake = new intake(
                (DcMotorEx) hardwareMap.dcMotor.get("intakeL"),
                (DcMotorEx) hardwareMap.dcMotor.get("intakeR")
        );

        Turret = new turret(
                hardwareMap.servo.get("turretL"),
                hardwareMap.servo.get("turretR"),
                (DcMotorEx) hardwareMap.dcMotor.get("launcherFront"),
                (DcMotorEx) hardwareMap.dcMotor.get("launcherRear")
        );

        ServoGate = new gate(
                hardwareMap.servo.get("gateL"),
                hardwareMap.servo.get("gateR"),
                hardwareMap.servo.get("indicatorLight")
        );

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            blackboard.put("BotPose", follower.pose());
            Scheduler.execute();

            // Continuously execute subsystem PID loops
            updateIntake();
            updateLauncher();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path shootPreload() {
        return Paths.line(start, shootPreload).linear(start, shootPreload);
    }

    public Path getGarden() {
        return Paths.curve(getGardenStart, getGardenControl1, getGarden).linear(getGardenStart, getGarden);
    }

    public Path shootGarden() {
        return Paths.line(getGarden, shootGarden)
                .heading(Interpolator.piecewise()
                        .until(0.3, Interpolator.constant(shootGardenSegment1Heading))
                        .until(1, Interpolator.linear(shootGardenSegment2Start, shootGardenSegment2End)));
    }

    public Path preparePush() {
        return Paths.line(preparePushStart, preparePush).linear(preparePushStart, preparePush);
    }

    public Path pushPartner() {
        return Paths.line(preparePush, pushPartner).constant(pushPartner);
    }
}
