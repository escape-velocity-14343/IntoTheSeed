package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.SquIDController;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

// override execute isfinished, constructor, initilize, end
public class SquIDDriveCommand extends CommandBase {
    public static Double pXDefault = null;
    public static Double pYDefault = null;
    public static Double pHDefault = null;

    private final SquIDController controllerX;
    private final SquIDController controllerY;
    private final SquIDController controllerH;
    private final PinpointSubsystem otos;
    private final MecanumDriveSubsystem drive;
    private Pose2D target;
    // default: 0.1"
    private double linearTolerance = 0.1;
    // default: 2°
    private double angularTolerance = 2;

    public SquIDDriveCommand(
            PinpointSubsystem otos,
            MecanumDriveSubsystem drive,
            SquIDController controllerX,
            SquIDController controllerY,
            SquIDController controllerH) {
        this.otos = otos;
        this.drive = drive;
        this.controllerX = controllerX;
        this.controllerY = controllerY;
        this.controllerH = controllerH;
        addRequirements(otos, drive);
    }

    public SquIDDriveCommand(
            PinpointSubsystem otos, MecanumDriveSubsystem drive, double pX, double pY, double pH) {
        this(otos, drive, new SquIDController(), new SquIDController(), new SquIDController());
        controllerX.setPID(pX);
        controllerY.setPID(pY);
        controllerH.setPID(pH);
        SquIDDriveCommand.pXDefault = pX;
        SquIDDriveCommand.pYDefault = pY;
        SquIDDriveCommand.pHDefault = pH;
    }

    public SquIDDriveCommand(PinpointSubsystem otos, MecanumDriveSubsystem drive) {
        this(otos, drive, pXDefault, pYDefault, pHDefault);
        if (pXDefault == null) {
            throw new NullPointerException("pXDefault was null");
        }
        if (pYDefault == null) {
            throw new NullPointerException("pYDefault was null");
        }
        if (pHDefault == null) {
            throw new NullPointerException("pHDefault was null");
        }
    }

    @Override
    public void execute() {
        Pose2d otosPose = otos.getPose();

        drive.driveFieldCentric(
                controllerX.calculate(target.getX(DistanceUnit.INCH), otosPose.getX()),
                controllerY.calculate(target.getY(DistanceUnit.INCH), otosPose.getY()),
                controllerH.calculate(target.getHeading(AngleUnit.DEGREES), otosPose.getHeading()));
    }

    @Override
    public boolean isFinished() {
        Pose2d otosPose = otos.getPose();

        double dx = (target.getX(DistanceUnit.INCH) - otosPose.getX());
        double dy = (target.getY(DistanceUnit.INCH) - otosPose.getY());
        double linearDistance = Math.sqrt(dx * dx + dy * dy);
        double angularDistance =
                Util.getAngularDifference(
                        target.getHeading(AngleUnit.DEGREES), otosPose.getRotation().getDegrees());

        return linearDistance < linearTolerance && angularDistance < angularTolerance;
    }

    public SquIDDriveCommand setTarget(Pose2D target) {
        this.target = target;
        return this;
    }

    /**
     * @param tolerance The linear tolerance in inches
     * @return <code>this</code>, for chaining
     */
    public SquIDDriveCommand setLinearTolerance(double tolerance) {
        this.linearTolerance = tolerance;
        return this;
    }

    /**
     * @param tolerance The angular tolerance in degrees
     * @return <code>this</code>, for chaining
     */
    public SquIDDriveCommand setAngularTolerance(double tolerance) {
        this.angularTolerance = tolerance;
        return this;
    }

    public static void setProportionalConstants(double pX, double pY, double pH) {
        pXDefault = pX;
        pYDefault = pY;
        pHDefault = pH;
    }
}
