package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.lib.DrivetrainSquIDController;
import org.firstinspires.ftc.teamcode.lib.SquIDController;
import org.firstinspires.ftc.teamcode.subsystems.BucketSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

@Config
public class BucketAlignCommand extends CommandBase {
    public static double targetXDistance = 10;
    public static double targetYDistance = 10;
    public static double targetHeading = -3.1415 / 4.0;
    public static double translationkP = DefaultGoToPointCommand.translationkP; // 0.008;
    public static double headingkP = 0.02;
    public static double xOffset = 0;
    public static double yOffset = 0;
    public boolean relocalize = false;

    private final MecanumDriveSubsystem mecanumDrive;
    private final BucketSensorSubsystem basketSensor;
    private final PinpointSubsystem pinpoint;
    private final SquIDController headingController;
    private final DrivetrainSquIDController dtController;
    private final double threshold;

    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;

    private BooleanSupplier interruptSupplier = () -> false;

    private Translation2d targetPoseOffset = new Translation2d();

    public double error;

    public BucketAlignCommand(
            MecanumDriveSubsystem mecanumDrive,
            BucketSensorSubsystem basketSensor,
            PinpointSubsystem pinpoint,
            double threshold) {
        this.mecanumDrive = mecanumDrive;
        this.basketSensor = basketSensor;
        this.pinpoint = pinpoint;
        addRequirements(mecanumDrive, basketSensor);
        headingController = new SquIDController();
        dtController = new DrivetrainSquIDController();
        this.threshold = threshold;

        xSupplier = () -> 0.0;
        ySupplier = () -> 0.0;
    }

    /**
     * Creates a never ending align command for testing
     *
     * @param mecanumDriveSubsystem the drive subsystem
     * @param bucketSensorSubsystem the sensor subsystem
     */
    public BucketAlignCommand(
            MecanumDriveSubsystem mecanumDriveSubsystem,
            BucketSensorSubsystem bucketSensorSubsystem,
            PinpointSubsystem pinpoint) {
        this(mecanumDriveSubsystem, bucketSensorSubsystem, pinpoint, -1.0);
    }

    @Override
    public boolean isFinished() {
        Log.i("%Autoscore", "Bucket align done");
        return (threshold > 0.0 && error < threshold) || interruptSupplier.getAsBoolean();
    }

    @Override
    public void execute() {
        Translation2d pose = new Translation2d(
                xSupplier.getAsDouble(),
                ySupplier.getAsDouble()
        ).rotateBy(new Rotation2d(-targetHeading));

        targetPoseOffset = pose;
        double x = targetXDistance + pose.getX();
        double y = targetYDistance + pose.getY();

        headingController.setPID(headingkP);
        dtController.setPID(translationkP);

        Pose2d p = dtController.calculate(
                AutoConstants.scorePos,
                pinpoint.getPose(),
                pinpoint.getVelocity()
        );
        double dtFactor = 1.0;
        double voltageMult = mecanumDrive.getAutoVoltageMult();
        mecanumDrive.driveFieldCentric(
                -p.getX() * dtFactor * voltageMult,
                -p.getY() * dtFactor * voltageMult,
                -headingController.calculate(targetHeading, pinpoint.getPose().getHeading()) * voltageMult
        );
        error = Math.hypot(y - basketSensor.getSensorLeft(), x - basketSensor.getSensorRight());
        Log.v("basket error", Double.toString(error));
    }

    public BucketAlignCommand withXySupplier(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        return this;
    }

    @Override
    public void end(boolean interrupted) {
        if (relocalize && !interrupted && !interruptSupplier.getAsBoolean()) {
            relocalizePinpoint();
        }
    }

    private void relocalizePinpoint() {
        if (!(Math.abs(pinpoint.getPose().getRotation().getDegrees() - AutoConstants.scorePos.getRotation().getDegrees()) < 2.5)) {
            return;
        }
        Pose2d newPose = new Pose2d(
                AutoConstants.scorePos.getX() - basketSensor.getSensorRight() + xOffset,
                AutoConstants.scorePos.getY() - basketSensor.getSensorLeft() + yOffset,
                new Rotation2d()
        );
        double distance = AutoConstants.scorePos.minus(newPose).getTranslation().getNorm();
        if (distance < 10.0) {
            pinpoint.setPosition(newPose.getX(), newPose.getY());
        }
    }

    public Command setInterrupt(BooleanSupplier booleanSupplier) {
        this.interruptSupplier = booleanSupplier;
        return this;
    }

    public Command whenClose(double activationDistance, Command command) {
        return alongWith(new WaitUntilCommand(() -> error < activationDistance).andThen(command));
    }
}
