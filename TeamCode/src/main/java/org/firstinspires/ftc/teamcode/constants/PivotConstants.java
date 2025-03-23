package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PivotConstants {
    public static double kPRetracted = 0.015;

    public static double kPExtended = 0.03;
    public static double kS = 0;
    public static double kGRetracted = 0;
    public static double kGFullyExtended = 0;

    public static double bottomLimit = 0.5;
    public static double topLimit = 86;
    public static double tolerance = 1.5;
    public static double direction = -1;
    public static boolean encoderInvert = true;
    public static double encoderOffset = 170.2;
    public static double outtakeExtendDegrees = 45;
    public static double retractDegrees = bottomLimit;

    public static double neutralPos = bottomLimit;
    public static double intakeReadyPos = 15;
    public static double intakePos = 10;

    public static double specimenIntakeAngle = topLimit;
    public static double specimenTopBarAngle = 63;

    public static double manualControlDeadband = 0.1;

    public static double bottomPMult = 1.5;
}

