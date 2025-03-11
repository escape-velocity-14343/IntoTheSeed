
package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PivotConstants {
    public static double kPRetracted = 0.03;
    public static double kPExtended = 0.05;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 0;

    public static double bottomLimit = 0.5;
    public static double topLimit = 86;
    public static double tolerance = 1.5;
    public static double direction = -1;
    public static boolean encoderInvert = true;
    public static double encoderOffset = -178.3;
    public static double outtakeExtendDegrees = 45;
    public static double hangDegrees = -10;
    public static double parkDegrees = 80;
    public static double retractDegrees = bottomLimit;
    public static double reversedRetractDegrees = 10;
    public static double frontOuttakePosition = 85;

    public static double neutralPos = bottomLimit;
    public static double intakeReadyPos = 17;
    public static double intakePos = 7.5;


    public static double specimenIntakeAngle = topLimit;
    public static double specimenTopBarAngle = 63;
    public static double l3hangtime = 5;
}
