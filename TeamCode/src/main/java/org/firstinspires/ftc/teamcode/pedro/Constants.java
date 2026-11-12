package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.872715987558441);
        c.yPodOffset.set(0.03653141691928773);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.backLeftName.set("leftBack");
        c.frontRightName.set("rightFront");
        c.backRightName.set("rightBack");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

        c.manualBrakeMode.set(true);
    });

    // see below
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.34054349039377474);
                Controller secondaryTranslationalForward = Controller.proportional(0.12582172021084875);
                Controller primaryTranslationalLateral = Controller.proportional(0.5439822811174096);
                Controller secondaryTranslationalLateral = Controller.proportional(0.20098691739862776);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012241519700593359));
                c.brake.set(Controller.proportionalFeedforward(0.010405291745504355));

                c.headingFeedback.set(Controller.proportional(5.106676971840961));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06997637499255248, 0.00332763498688731));

                c.linearBrakeCoefficients.set(Matrix.diag(0.1308848900550059, 0.003775590660182452));
                c.quadraticBrakeCoefficients.set(Matrix.diag(9.5864506187827E-4, 2.0418417417312983E-4));

                c.maxAchievableForwardVelocity.set(78.64091806497126);
                c.maxAchievableStrafeVelocity.set(66.01892357764709);
                c.naturalForwardDeceleration.set(39.64644974942244);
                c.naturalStrafeDeceleration.set(70.46611886519302);

                c.maxBrakingPower.set(0.25);
                c.maxDecelerationScale.set(0.9);
                //c.maxPathSpeed.set(60.0);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

}