package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PivotConstants {
    public static double maxVelocity = 4;
    public static double maxAcceleration = 4;

    public static double kPRetracted = -0.03;
    public static double kPExtended = -0.05;
    public static double kS = 0;
    public static double kD = -0.2;
    public static double kGRetracted = 0.13;
    public static double debugGain = 0;
    public static double kGFullyExtended = 0.35;

    public static double maxPivotVelocity = 1.0;

    public static double bottomLimit = 0.5;
    public static double topLimit = 86;
    public static double stallTopLimit = 88.5;
    public static double tolerance = 3;
    public static double direction = -1;
    public static boolean encoderInvert = true;
    public static double encoderOffset = -56.94;
    public static double outtakeExtendDegrees = 30;
    public static double autoOuttakeExtendDegrees = 10;
    public static double retractDegrees = bottomLimit;

    public static double neutralPos = 25;
    public static double intakeReadyPos = 15;
    public static double intakePos = 10;

    public static double specimenIntakeAngle = topLimit;
    public static double specimenTopBarAngle = 63;

    public static double manualControlDeadband = 0.1;

    public static double bottomPMult = 1.25;
    public static double hangReady = 60.0;
    public static double powerCutAngle = 30;


}

