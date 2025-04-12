package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.commands.custom.BucketRelocalizeCommand;
import org.firstinspires.ftc.teamcode.commands.custom.DrivetrainBrakeCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IVKCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.InterruptCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SequentialIVKCommand;
import org.firstinspires.ftc.teamcode.commands.custom.StoreCoarsePositionCommand;
import org.firstinspires.ftc.teamcode.commands.custom.StoreFinePositionCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TimeoutCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WaitUntilStabilizedCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IVKConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.SampleMovementOptimizer;
import org.firstinspires.ftc.teamcode.lib.SamplePoseStorage;
import org.firstinspires.ftc.teamcode.lib.path.spline.CubicBezier;
import org.firstinspires.ftc.teamcode.subsystems.BucketSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TargetingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AutoSubCycle extends SequentialCommandGroup {

    public AutoSubCycle(VisionSubsystem vision, PivotSubsystem pivot, ExtensionSubsystem extension, SamplePoseStorage storage, DefaultDualMoveCommand dmc, MecanumDriveSubsystem drive, PinpointSubsystem pinpoint, IntakeSubsystem intake, WristSubsystem wrist, TurretSubsystem turret, TargetingSubsystem target, BucketSensorSubsystem bucketSensors) {

        super(
                // drive to sub
                dmc.setGVF().andThen(
                        new InterruptCommand(
                                new GVFWithDefaultCommand(dmc.getGvfc(), 3.0, 10.0, () -> {

                                    Pose2d intermediate = SampleMovementOptimizer.getIntermediatePoint(storage.getCoarsePosition(), -18.0, 35.0, 25.0);
                                    Pose2d end = SampleMovementOptimizer.getClosestPoint(storage.getCoarsePosition(), -18.0, 35.0, 25.0);

                                    return new CubicBezier[]{new CubicBezier(AutoConstants.cycleScorePos.getX(), AutoConstants.cycleScorePos.getY(),
                                            -42, 53,
                                            intermediate.getX(), intermediate.getY(),
                                            end.getX(), end.getY())};
                                }
                                ),

                                () -> pivot.isDone() && false //vision.getPossibilityForSampleExistingInThisGivenMomentOfTimeAndSpace()
                        )
                ).alongWith(
                        new ParallelCommandGroup(
                                new WaitUntilCommand(() -> pinpoint.getPose().relativeTo(AutoConstants.scorePos).getTranslation().getNorm() > 2.5).andThen(
                                        new InterruptCommand(new ExtendCommand(extension, 0.0), () -> extension.getCurrentInches() < SlideConstants.pivotDownExtension)
                                ),
                                new WristCommand(wrist, IntakeConstants.groundPos),
                                new InstantCommand(() -> vision.setCam(false)),
                                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1)
                        ).andThen(
                                new WristCommand(wrist, IntakeConstants.groundPos - 0.115),
                                new InterruptCommand(
                                        new PivotCommand(pivot, PivotConstants.bottomLimit),
                                        () -> pivot.getCurrentPosition() < 20.0
                                ).alongWith(
                                        new TurretCommand(turret, 0.0),
                                        new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 0)
                                ),
                                new InterruptCommand(
                                        new IVKCommand(SlideConstants.submersibleIntakeMidExtension, IVKCommand.intakeReadyY, extension, pivot, 0.8),
                                        () -> pivot.getPivotVelocity() < AutoConstants.autoscoreMaxPivotVel
                                )
                        )

                ),

                // stabilize
                new DrivetrainBrakeCommand(dmc),
                new WaitUntilStabilizedCommand(pinpoint).alongWith(
                        new WristCommand(wrist, IntakeConstants.toptakePos - 0.1)
                ),
                new InstantCommand(drive::clearBrake),
                dmc.setP2P(),
                new InstantCommand(() -> dmc.getGtpc().setTarget(pinpoint.getPose())),

                // target and go to sample
                new TimeoutCommand(
                        new StoreFinePositionCommand(vision, storage, pinpoint, pivot, extension, turret),
                        400
                ),
                new WaitUntilCommand(() -> dmc.getGtpc().isDone()).alongWith(
                        new WaitUntilCommand(extension::isClose)
                ).deadlineWith(
                        new PerpetualCommand(new StoreFinePositionCommand(vision, storage, pinpoint, pivot, extension, turret)),
                        new PerpetualCommand(
                                new ExtendCommand(extension, storage::getNewExtension) {
                                    @Override
                                    public void execute() {
                                        super.execute();
                                        extension.setTargetInches(storage.getNewExtension());
                                        Log.v("Auto Sub Cycle", "Extension Target: " + storage.getNewExtension());
                                        Log.v("Auto Sub Cycle", "Extension: " + extension.getCurrentInches());
                                    }
                                }
                        ),
                        new PerpetualCommand(
                                new GoToPointWithDefaultCommand(storage::getFinePosition, dmc.getGtpc(), 0.5, 2) {
                                    @Override
                                    public void execute() {
                                        dmc.getGtpc().setTarget(storage.getFinePosition());
                                        Log.v("Auto Sub Cycle", "Heading Target: " + storage.getFinePosition().getRotation().getDegrees());
                                        Log.v("Auto Sub Cycle", "Heading: " + pinpoint.getPose().getRotation().getDegrees());
                                    }
                                }
                        )
                ),
                new WristCommand(wrist, IntakeConstants.toptakePos),

                /*new GoToPointWithDefaultCommand(storage::getFinePosition, dmc.getGtpc(), 0.5, 2).alongWith(
                        new ExtendCommand(extension, storage::getNewExtension),

                ),
                //new WaitUntilStabilizedCommand(pinpoint),
                new InstantCommand(() -> vision.setCam(true)),*/

                // What we need to do next:
                // Wait Until Stabilized pitch
                // Use the non-supplier subPos Command because there's no reason to use a supplier, we're
                // continually supplying the same value over and over
                // We can keep the timeout
                // Potentially what we can do is create a new stabilizedSubCommand which splits the IVK
                // Into 1 inch Y value increments, and uses wait for stabilized internally between each iteration

                // intake
                new TimeoutCommand(
                        new SubPosCommand(extension, wrist, intake, pivot, () -> Math.cos(Math.toRadians(pivot.getCurrentPosition())) * extension.getCurrentInches()), 200
                ),
                new ParallelRaceGroup(
                        new WaitCommand(500),
                        new WaitUntilCommand(intake::proxClose)
                ),

                // go to score
                dmc.setGVF(),
                new InterruptCommand(
                        new GVFWithDefaultCommand(dmc.getGvfc(), 5, 10, () -> new CubicBezier[]{new CubicBezier(pinpoint.getPose().getX(), pinpoint.getPose().getY(),
                                -24, 40,
                                -42, 53,
                                AutoConstants.cycleScorePos.getX(), AutoConstants.cycleScorePos.getY())}
                        ).reverseHeading(),
                        () -> AutoConstants.cycleScorePos.minus(pinpoint.getPose()).getTranslation().getNorm() < 5.0
                ).alongWith(
                        new InterruptCommand(
                                new RetractCommand(wrist, pivot, extension, turret, intake),
                                () -> pinpoint.getPose().getY() - extension.getCurrentInches() > 20
                        ).andThen(
                                new BucketPosCommand(extension, pivot, wrist, turret, false)
                        ),
                        new TimeoutCommand(new StoreCoarsePositionCommand(vision, storage, pinpoint), 500)
                ),
                new WaitUntilCommand(intake::stable),
                new IntakeControlCommand(intake, IntakeConstants.openPos, 0)
        //new GoToPointWithDefaultCommand(AutoConstants.scorePos, dmc.getGtpc()),
        //new BucketRelocalizeCommand(bucketSensors, pinpoint, 0.1).alongWith(new WaitCommand(100)),
        //dmc.setP2P()
        );

    }


}
