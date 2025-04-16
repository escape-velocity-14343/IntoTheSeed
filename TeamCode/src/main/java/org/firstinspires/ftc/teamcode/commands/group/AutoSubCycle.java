package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.commands.custom.BucketRelocalizeCommand;
import org.firstinspires.ftc.teamcode.commands.custom.DrivetrainBrakeCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IVKCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClosingCommand;
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
import org.firstinspires.ftc.teamcode.lib.SlideKinematics;
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

import java.sql.Time;

public class AutoSubCycle extends SequentialCommandGroup {

    public AutoSubCycle(VisionSubsystem vision, PivotSubsystem pivot, ExtensionSubsystem extension, SamplePoseStorage storage, DefaultDualMoveCommand dmc, MecanumDriveSubsystem drive, PinpointSubsystem pinpoint, IntakeSubsystem intake, WristSubsystem wrist, TurretSubsystem turret, TargetingSubsystem target, BucketSensorSubsystem bucketSensors) {

        super(
                // drive to sub
                dmc.setGVF().andThen(
                        new InterruptCommand(
                                new GVFWithDefaultCommand(dmc.getGvfc(), 3.0, 10.0, () -> {

                                    Pose2d intermediate = SampleMovementOptimizer.getIntermediatePoint(storage.getCoarsePosition(), -18.0, 45.0, SlideConstants.submersibleIntakeMidExtension);
                                    Pose2d end = SampleMovementOptimizer.getClosestPoint(storage.getCoarsePosition(), -18.0, 45.0, SlideConstants.submersibleIntakeMidExtension);

                                    return new CubicBezier[]{new CubicBezier(AutoConstants.scorePos.getX(), AutoConstants.scorePos.getY(),
                                            -52, 48,
                                            intermediate.getX(), intermediate.getY(),
                                            end.getX(), end.getY())};
                                }
                                ),

                                () -> pivot.isDone() && false //vision.getPossibilityForSampleExistingInThisGivenMomentOfTimeAndSpace()
                        )
                ).alongWith(
                        new ParallelCommandGroup(
                                new WaitUntilCommand(() -> pinpoint.getPose().relativeTo(AutoConstants.scorePos).getTranslation().getNorm() > 6.0).andThen(
                                        new InterruptCommand(new ExtendCommand(extension, 0.0), () -> extension.getCurrentInches() < SlideConstants.pivotDownExtension - 15.0)
                                ),
                                new InstantCommand(() -> vision.setCam(false))
                        ).andThen(
                                new WristCommand(wrist, IntakeConstants.halfFoldPos),
                                new InterruptCommand(
                                        new PivotCommand(pivot, PivotConstants.bottomLimit + 15),
                                        () -> pivot.getCurrentPosition() < 20.0
                                ).alongWith(
                                        new TurretCommand(turret, 0.0),
                                        new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1)
                                ),
                                new InterruptCommand(
                                        SlideKinematics.getIVKCommand(extension, pivot, new Translation2d(SlideConstants.submersibleIntakeMidExtension, IVKConstants.clawIntakeIVKHeight+6.0), 0.8),
                                        () -> pivot.getPivotVelocity() < AutoConstants.autoscoreMaxPivotVel
                                )
                        )

                ),

                // stabilize
                new DrivetrainBrakeCommand(dmc),
                new WaitUntilStabilizedCommand(pinpoint).alongWith(
                        new WristCommand(wrist, IntakeConstants.halfFoldPos)
                ),
                dmc.setP2P(),
                new InstantCommand(() -> dmc.getGtpc().setTarget(pinpoint.getPose())),

                // target and go to sample
                new TimeoutCommand(
                        new StoreFinePositionCommand(vision, storage, pinpoint, pivot, extension, turret),
                        100
                ),
                new WaitUntilCommand(extension::isClose).alongWith(
                        new WaitUntilCommand(dmc.getGtpc()::isDone)
                ).deadlineWith(
                        new StoreFinePositionCommand(vision, storage, pinpoint, pivot, extension, turret).perpetually(),
                        new RunCommand(() -> dmc.getGtpc().setTarget(storage.getFinePosition())),
                        new RunCommand(() -> extension.setTargetInches(SlideKinematics.getIVKClawPos(new Translation2d(storage.getNewExtension(), IVKConstants.clawIntakeIVKHeight)).getX()*IVKConstants.extensionScalar), extension)
                ),
                new WristCommand(wrist, IntakeConstants.toptakePos),
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1),
                new TimeoutCommand(new PivotCommand(pivot, () -> SlideKinematics.getIVKClawPos(new Translation2d(storage.getNewExtension(), IVKConstants.clawIntakeIVKHeight)).getRotation().getDegrees()), 700),
                new TimeoutCommand(
                        new ParallelCommandGroup(
                                new IntakeClosingCommand(intake, IntakeConstants.slightOpenPos, 1),
                                new WaitUntilCommand(intake::proxClose))
                        , 500),
                new IntakeControlCommand(intake, IntakeConstants.closedPos, 1),
                new InstantCommand(drive::clearBrake),

                // go to score
                dmc.setGVF(),
                new InterruptCommand(
                        new GVFWithDefaultCommand(dmc.getGvfc(), 5, 10, () -> new CubicBezier[]{new CubicBezier(
                                pinpoint.getPose().getX(), pinpoint.getPose().getY(),
                                -30 - 0.5 * pinpoint.getPose().getX() + 10, 35,
                                -52, 48,
                                AutoConstants.scorePos.getX()+3, AutoConstants.scorePos.getY()+2
                        )}
                        ).reverseHeading(),
                        () -> AutoConstants.scorePos.minus(pinpoint.getPose()).getTranslation().getNorm() < 5.0
                ).alongWith(
                        new InterruptCommand(
                                new RetractCommand(wrist, pivot, extension, turret, intake, true),
                                () -> pinpoint.getPose().getY() - extension.getCurrentInches() > 16
                        ).andThen(
                                new BucketPos2Command(extension, pivot, wrist, turret, false)
                        ),
                        new TimeoutCommand(new StoreCoarsePositionCommand(vision, storage, pinpoint), 500),
                        new WaitUntilCommand(() -> AutoConstants.scorePos.minus(pinpoint.getPose()).getTranslation().getNorm() < AutoConstants.fastDropDistance).andThen(
                                new IntakeControlCommand(intake, IntakeConstants.openPos, 0)
                        )
                ),
                new DrivetrainBrakeCommand(dmc),
                new WaitCommand(100),
                new InstantCommand(drive::clearBrake)

                //new GoToPointWithDefaultCommand(AutoConstants.scorePos, dmc.getGtpc()),
                //new BucketRelocalizeCommand(bucketSensors, pinpoint, 0.1).alongWith(new WaitCommand(100)),
                //dmc.setP2P()
        );

    }


}
