package org.firstinspires.ftc.teamcode.team.libraries;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class LocalizationUtils {
    public static double getDistancePose2D(Pose2D Robot, Pose2D Target) {
        double RobotX = Robot.getX(DistanceUnit.MM);
        double RobotY = Robot.getY(DistanceUnit.MM);
        double TargetX = Target.getX(DistanceUnit.MM);
        double TargetY = Target.getY(DistanceUnit.MM);

        return Math.abs(Math.sqrt(Math.pow(TargetX - RobotX, 2) + Math.pow(TargetY - RobotY, 2)));
    }
    public static double getDistancePose(Pose Robot, Pose Target) {
        double RobotX = Robot.x();
        double RobotY = Robot.y();
        double TargetX = Target.x();
        double TargetY = Target.y();

        return Math.abs(Math.sqrt(Math.pow(TargetX - RobotX, 2) + Math.pow(TargetY - RobotY, 2)));
    }

    public static double getAnglePose2D(Pose2D Robot, Pose2D Target) {
        double RobotX = Robot.getX(DistanceUnit.MM);
        double RobotY = Robot.getY(DistanceUnit.MM);
        double TargetX = Target.getX(DistanceUnit.MM);
        double TargetY = Target.getY(DistanceUnit.MM);

        double RobotHeading = Robot.getHeading(AngleUnit.DEGREES);

        double BotAngle = Math.toDegrees(Math.atan2(-(TargetY - RobotY), (TargetX - RobotX)));

        double difference = -BotAngle - RobotHeading;

        while (difference > 180) difference -= 360;
        while (difference <= -180) difference += 360;

        return difference;
    }

    public static double getAngle(Pose Robot, Pose Target) {
        double RobotX = Robot.x();
        double RobotY = Robot.y();
        double TargetX = Target.x();
        double TargetY = Target.y();

        double RobotHeading = Math.toDegrees(Robot.heading());

        double BotAngle = Math.toDegrees(Math.atan2(-(TargetY - RobotY), (TargetX - RobotX)));

        double difference = -BotAngle - RobotHeading;

        while (difference > 180) difference -= 360;
        while (difference <= -180) difference += 360;

        return difference;
    }

    public static double DegreesRollover(double input){
        while (input > 180) input -= 360;
        while (input <= -180) input += 360;

        return input;
    }
}
