package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.Localizer;
import org.firstinspires.ftc.teamcode.lib.drivers.GoBildaPinpoint;

@Config
public class PinpointSubsystem extends SubsystemBase implements Localizer {
    GoBildaPinpoint pinpoint;
    Pose2D pose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    GoBildaPinpoint.DeviceStatus deviceStatus = GoBildaPinpoint.DeviceStatus.NOT_READY;

    @Deprecated public static double yawScalar = 1;
    public static boolean flipX = true;
    public static boolean flipY = false;
    public static double xEncOffset = 65;
    public static double yEncOffset = 85;

    private Pose2D lastGoodPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    public PinpointSubsystem(HardwareMap hMap) {

    }

    @Override
    public void periodic() {

    }

    private Pose2D getSDKPose() {
        return pose;
    }

    public int[] getEncoderCounts() {
        return new int[] {0,0};
    }

    public Pose2d getPose() {
        return new Pose2d(
                getSDKPose().getX(DistanceUnit.INCH),
                getSDKPose().getY(DistanceUnit.INCH),
                Rotation2d.fromDegrees(getSDKPose().getHeading(AngleUnit.DEGREES)));
    }

    public Pose2d getVelocity() {

        return new Pose2d(
                0,0,new Rotation2d(0));
    }

    public void reset() {

        lastGoodPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    }

    /**
     * @param x In inches.
     * @param y In inches.
     */
    public void setPosition(double x, double y) {

    }

    public void setHeading(double x, double y, double headingDegrees){

    }

    public boolean isDoneCalibration() {

        return false;
    }

    public void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 7;

        c.setStrokeWidth(1);
        c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);

        Vector2d halfv =
                new Vector2d(t.getRotation().getCos(), t.getRotation().getSin())
                        .times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = new Vector2d(t.getX(), t.getY()).plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.getX(), p1.getY(), p2.getX(), p2.getY());
    }

    /** Warning - will completely break position!! */
    public void resetYaw() {
    }
}
