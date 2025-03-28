package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VisionConstants {
    //1280 x 800
    public static String slideCameraName = "slidecamera";
    public static String chassisCameraName = "chassiscamera";
    public static int width = 1280;
    public static int height = 720;

    public static double cx = 338.083 / 2;
    public static double cy = 218.771 / 2;
    public static double focalL = 491.437 / 2;

    public static double visionKP = 0.0025;
    public static double yOffset = 120;
    public static double xOffset = 250;
    public static double velocityEnd = 0.001;
    public static double maxPower = 0.3;

    public static double visionEndThreshold = 10;

    public static boolean reverseX = true;
    public static boolean reverseY = false;

    //Supported resolutions 1280 720
    // 800 600
    // 960 540
    // 640 840

    public static int defaultWhiteBalance = 3000;
}
