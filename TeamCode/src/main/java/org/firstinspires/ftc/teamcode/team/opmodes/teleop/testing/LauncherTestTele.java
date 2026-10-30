package org.firstinspires.ftc.teamcode.team.opmodes.teleop.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.team.libraries.GamepadButton;
import org.firstinspires.ftc.teamcode.team.subsystems.turret;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "LauncherTestTele", group = "Linear OpMode")
public class LauncherTestTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry(); //I followed you

        panelsTelemetry.update(telemetry);

        GamepadButton launchButton = new GamepadButton(gamepad1, GamepadButton.gamepadKeys.RIGHT_BUMPER);

        turret Turret = new turret(
                hardwareMap.servo.get("turretL"),
                hardwareMap.servo.get("turretR"),
                (DcMotorEx) hardwareMap.dcMotor.get("launcherFront"),
                (DcMotorEx) hardwareMap.dcMotor.get("launcherRear")
        );

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive())
        {
            Turret.launcherSetVel(turret.fallbackVelocity);

            telemetry.addData("LauncherTarget", turret.fallbackVelocity);

            telemetry.addData("LauncherVel", Turret.launcherGetVel());
            telemetry.addData("LauncherP", Turret.launcherGetP());
            telemetry.addData("LauncherI", Turret.launcherGetI());
            telemetry.addData("LauncherD", Turret.launcherGetD());

            panelsTelemetry.addData("LauncherTarget", turret.fallbackVelocity);

            panelsTelemetry.addData("LauncherVel", Turret.launcherGetVel());
            panelsTelemetry.addData("LauncherP", Turret.launcherGetP());
            panelsTelemetry.addData("LauncherI", Turret.launcherGetI());
            panelsTelemetry.addData("LauncherD", Turret.launcherGetD());

            telemetry.update();
            panelsTelemetry.update(telemetry);
        }
    }
}

