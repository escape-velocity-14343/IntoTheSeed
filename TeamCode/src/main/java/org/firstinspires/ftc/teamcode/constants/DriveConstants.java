package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 0;

    public static boolean useFieldCentric = false;
    public static boolean drawRobot = true;

    // pto
    public static double ptoRightEngagedPos = 0.3;
    public static double ptoRightDisengagedPos = 0.6;


    public static double ptoLeftEngagedPos = 0.15;
    public static double ptoLeftDisengagedPos = 0.45;

    public static double strafeMultiplier = 1.1;

    public static double forwardMotorMultiplier = 0.35;
    public static double ptoPowerFac = 0.5;
}
