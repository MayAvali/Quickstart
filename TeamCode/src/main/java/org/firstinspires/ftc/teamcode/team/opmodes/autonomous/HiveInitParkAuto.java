package org.firstinspires.ftc.teamcode.team.opmodes.autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.team.libraries.PositionBank;
import org.firstinspires.ftc.teamcode.team.subsystems.gate;
import org.firstinspires.ftc.teamcode.team.subsystems.intake;
import org.firstinspires.ftc.teamcode.team.subsystems.turret;

import java.util.List;

@Autonomous(name = "1. Autonomous HiveInit PARK", group = "Autonomous OpMode")
public class HiveInitParkAuto extends LinearOpMode {

    final Pose InitPose = PositionBank.HiveInitPosition;
    final Pose ParkPose = PositionBank.LoadingZoneParkPosition;

    Follower follower; // Add this
    private Path park(){
        return line(InitPose, ParkPose).linear(InitPose, ParkPose);
    }

   private Command autoRoutine() {
        return sequential(
                // Add mechanism commands here.
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

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

        follower = Constants.create(hardwareMap);

        follower.setPose(InitPose);

        waitForStart();

        schedule(autoRoutine());

        if (isStopRequested()) return;
        while(opModeIsActive()) {
            blackboard.put("BotPose", follower.pose());

            Turret.launcherSetVel(1500);

            follower.update();
            Scheduler.execute();
            // add your other methods needed in the loop here

            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Follower Mode", follower.mode());
            telemetry.update();
        }

    }
}
