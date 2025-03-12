package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {

    // wrist constants
    public static double foldedPos = 0.35;
    public static double halfFoldPos = 0.47;
    public static double groundPos = 0.56;
    public static double scoringPos = 0.3;
    public static double specimenScoringPos = 0.4;
    public static double scoringPosReversed = 0.75;
    public static double parkPos = 0.45;
    public static double bucketRetractPos = 0.05;
    public static double specimenReadyPos = 0.63;
    public static double intakeReadyPos = 0.8;
    public static double toptakePos = 0.6;

    // wrist command constants
    public static double timeMultiplier = 0.27;

    // claw constants
    // fronttake
    public static double openPos = 0.6;
    public static double closedPos = 0.9;
    public static double singleIntakePos = 0.85;


    public static double clawOffset = -0.1;

    // turret constants
    public static double minAngle = -130;
    public static double maxAngle = 130;

    // randomly worked from old folded pos
    public static double rightAnglePos = 0.86;


    // auto constants
    public static double autoOuttakeSpeed = -0.15;
    public static double autoIntakeSpeed = 1;
    public static double autoIntakeClawLerp = 0.5;

    // auto heading alignment
    public static double autoAlignP = -0.3;
    public static double autoAlignTol = 10;

    // global offset
    // 1 tick of skip = 0.05 position
    public static double wristOffset = -0.2;

    public static double visionSizeWeight = 1;
    public static double intakeSensorVoltageThres = 0.25;

    public static double spitToBackMs = 100;
}
