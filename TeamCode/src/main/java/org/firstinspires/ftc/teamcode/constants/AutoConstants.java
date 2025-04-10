package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.canvas.Rotation;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

@Config
public class AutoConstants {

    public static Pose2d scorePos = new Pose2d(-61, 59, Rotation2d.fromDegrees(-45));
    public static Pose2d cycleScorePos = new Pose2d(-62, 63, Rotation2d.fromDegrees(-30));
    public static Pose2d scorePosOffset = new Pose2d(-62,58.5, Rotation2d.fromDegrees(-45));

    public static double subBarrierY = 18.5;
    public static double closeToBucketDistance = 1.0;

    public static double autoscoreMaxVel = 8.5;
    public static double stabilziedMaxAngVel = 15;

    // velocity necessary to skip to next point
    public static double stallVelocity = 0.5;
    public static double spikeExtensionSpeed = 0.5;
    public static double spike1ExtensionLength = 8;
    public static double spike2ExtensionLength = 7;
    public static double spike3ExtensionLength = 9;

    public static double clawCloseDistance = 1;
    public static double extendedWhilePivotOffset = 2;

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
