package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SlideConstants {
    public static double kP = 0.004;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 0.13;
    public static double ticksPerInch = 54.9;
    public static double maxExtension = 31;

    public static double bucketPos = 31;
    public static double lowBucketPos = 11.5;

    public static double minExtension = 0;
    public static double direction = 1;
    public static double tolerance = 1;
    public static double alertCurrent = 4;

    /** Feedforward value that is multiplied by <code>Math.cos(slideAngle)</code> */
    public static double FEEDFORWARD_bottom = 0.07;

    public static double FEEDFORWARD_top = 0.12; // TUNED VALUE

    public static double submersibleIntakeMinExtension = 10;
    public static double submersibleIntakeMaxExtension = 20;

    public static double specimenRaisePosition = 0;
    public static double specimenHighRaisePosition = 13;
    public static double specimenHookPosition = 16.75;

    public static double millisPerInch = 5 * ticksPerInch;

    public static double highExtendInches = 1.5;
    public static double extendedThreshold = 8;
}
