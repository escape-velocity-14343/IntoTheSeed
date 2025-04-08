package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BucketSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

@Config
public class BucketRelocalizeCommand extends CommandBase {
    public static double correctionFactor = 0.956701;
    public static double offsetX = 7.75;
    public static double offsetY = 10.75;
    public static double relocalizationMaxDistance = 10.0;
    public static double relocalizationMaxAngleDistance = 10.0;
    BucketSensorSubsystem bucketSensor;
    PinpointSubsystem pinpoint;

    private double timeoutSeconds = -1.0;
    private boolean canFinish = false;
    private boolean dryRun = false;
    private final ElapsedTime timeout = new ElapsedTime();
    private Telemetry telemetry;

    public BucketRelocalizeCommand(BucketSensorSubsystem bucketSensor, PinpointSubsystem pinpoint, double timeoutSeconds) {
        this.bucketSensor = bucketSensor;
        this.pinpoint = pinpoint;
        this.timeoutSeconds = timeoutSeconds;
        //addRequirements(bucketSensor, pinpoint);
    }

    @Override
    public void initialize() {
        timeout.reset();
    }

    @Override
    public void execute() {
        double theta = pinpoint.getPose().getRotation().getRadians();
        double headingCorrectionFactor = Math.cos(theta + Math.PI / 4.0);
        double xInches = bucketSensor.getSensorLeft() / correctionFactor * headingCorrectionFactor - 72 + offsetX;
        double yInches = 72 - bucketSensor.getSensorRight() / correctionFactor * headingCorrectionFactor - offsetY;
        if (getTelemetry() != null) {
            getTelemetry().addData("bucket x", xInches);
            getTelemetry().addData("bucket y", yInches);
            getTelemetry().addData("raw bucket x", bucketSensor.getSensorLeft());
            getTelemetry().addData("raw bucket y", bucketSensor.getSensorRight());
        }
        double distance = new Translation2d(pinpoint.getPose().getX(), pinpoint.getPose().getY()).getDistance(new Translation2d(xInches, yInches));
        // less than, not greater than because both are cosined
        if (headingCorrectionFactor < Math.cos(Math.toRadians(relocalizationMaxAngleDistance))) {
            Log.i("bucket relocalize failed", String.format("Didn't relocalize at time %s since heading was too far off, will make second attempt", timeout.seconds()));
        } else if (distance > relocalizationMaxDistance) {
            Log.i("bucket relocalize failed", String.format("Didn't relocalize at time %s since measured position was %.3f inches off, will make second attempt", timeout.seconds(), distance));
            Log.i("bucket relocalize failed", String.format("pinpoint: %.3f %.3f | ultrasonics: %.3f %.3f", pinpoint.getPose().getX(), pinpoint.getPose().getY(), xInches, yInches));
        } else {
            if (!isDryRun()) {
                pinpoint.setPosition(xInches, yInches);
                canFinish = true;
            }
            Log.i("bucket relocalize success", String.format("relocalized at time %.3f to %s %s. Pinpoint was %.3f inches off", timeout.seconds(), xInches, yInches, distance));
        }
    }

    @Override
    public boolean isFinished() {
        return !isDryRun() && (canFinish || timeout.seconds() > timeoutSeconds && timeoutSeconds > 0.0);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            return;
        }

        if (!canFinish) {
            Log.i("bucket relocalize failed", "Bucket relocalize timed out");
        } else {
            Log.i("bucket relocalize success", "Bucket relocalize finished with success");
        }
    }

    public boolean isDryRun() {
        return dryRun;
    }

    public void setDryRun(boolean dryRun) {
        this.dryRun = dryRun;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
}
