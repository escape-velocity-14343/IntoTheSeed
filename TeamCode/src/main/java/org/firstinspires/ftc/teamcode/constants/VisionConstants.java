package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Vector2d;

@Config
public class VisionConstants {
    //1280 x 800
    public static String slideCameraName = "slide";
    public static String chassisCameraName = "chassis";
    //QVGA
    public static int width = 320;
    public static int height = 240;
    public static int minHeight = 0;

    public static double cx = 338.083 / 2;
    public static double cy = 218.771 / 2;
    public static double focalL = 491.437 / 2;

    public static double visionKP = 0.0025;
    public static double yOffset = 120;
    public static double xOffset = 250;
    public static double velocityEnd = 0.001;
    public static double maxPower = 0.2;

    public static double visionEndThreshold = 10;
    public static double minExtensionYOffset = 3.5;

    public static boolean reverseX = true;
    public static boolean reverseY = false;
    public static double minSampleArea = 2000;

    public static double submersibleIntakeWeightThresholdInches = 15;

    //Supported resolutions 1280 720
    // 800 600
    // 960 540
    // 640 840

    public static int defaultWhiteBalance = 3000;

    public static double chassisX = 160;
    public static double chassisY = 150;
    public static Vector2d target = new Vector2d(chassisX, chassisY);
    public static int targetMiddle = 250;
    public static double distanceScalar = 0.9;
}
