package org.firstinspires.ftc.teamcode.lib;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

@Config
public class RobotSlidePnP {
    double cx,cy,fl;
    public static double rx = 0;
    public static double ry = 0;
    public static double rz = 0;
    public static double rp = 0; //robot xyz, robot pitch
    Pose2d fieldPos;

    public RobotSlidePnP(double cx, double cy, double fl) {
        this.cx = cx;
        this.cy = cy;
        this.fl = fl;
    }
    public Translation2d getRobotCentricTranslation(int px, int py) {
        double gt = Math.atan((py-cy)/fl);
        Log.v("camera centric angle", gt + "");
        double theta = rp-gt;
        Log.v("ground centric angle", theta + "");
        double gd = rz / Math.tan(theta);
        Log.v("ground distance", gd + "");
        double dist = Math.hypot(gd, rz);
        Log.v("sample distance", dist + "");
        double pd = Math.cos(gt)*dist;
        Log.v("plane distance", pd + "");
        double gx = (px-cx)*pd/fl;
        Log.v("ground horizontal distance", gx + "");
        // gd is backwards for some reason
        return new Translation2d(-gd + rx, ry-gx);
    }

    public Translation2d getFieldCoordinates(int px, int py, Pose2d robotPose) {

        Translation2d rctrans = getRobotCentricTranslation(px, py);

        return robotPose.getTranslation().plus(rctrans.rotateBy(robotPose.getRotation()));
    }


}
