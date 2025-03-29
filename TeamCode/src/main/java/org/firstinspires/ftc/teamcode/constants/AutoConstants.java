package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.canvas.Rotation;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

@Config
public class AutoConstants {

    public static Pose2d scorePos = new Pose2d(-60.5, 59.5, Rotation2d.fromDegrees(-45));
    public static Pose2d scorePosOffset = new Pose2d(-62,58.5, Rotation2d.fromDegrees(-45));

    public static double subBarrierY = 24;

    public static double autoscoreMaxVel = 8.5;

    // velocity necessary to skip to next point
    public static double stallVelocity = 0.5;
    public static double spikeExtensionSpeed = 0.5;
    public static double spikeExtensionLength = 10;

    // Milliseconds
    public static long outtakeTimeout = 1300;

    public static double autoscoreMaxPivotVel = 1.0;

    //Voltage Pause
    public static double baseVoltage = 12.5;

    public enum Alliance {
        RED,
        BLUE
    }

    public static Alliance alliance = Alliance.BLUE;
}
