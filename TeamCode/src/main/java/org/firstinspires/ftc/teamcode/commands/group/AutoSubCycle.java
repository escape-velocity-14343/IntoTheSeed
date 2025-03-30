package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.commands.custom.CoarseAlignCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.FineAlignCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IVKCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.InterruptCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.RunIfCommand;
import org.firstinspires.ftc.teamcode.commands.custom.StoreCoarsePositionCommand;
import org.firstinspires.ftc.teamcode.commands.custom.StoreFinePositionCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TimeoutCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.SampleMovementOptimizer;
import org.firstinspires.ftc.teamcode.lib.SamplePoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.sql.Time;

public class AutoSubCycle extends SequentialCommandGroup {

    public AutoSubCycle(VisionSubsystem vision, PivotSubsystem pivot, ExtensionSubsystem extension, SamplePoseStorage storage, DefaultGoToPointCommand gtpc, MecanumDriveSubsystem drive, PinpointSubsystem pinpoint, IntakeSubsystem intake, WristSubsystem wrist, TurretSubsystem turret) {

        super(
                new InstantCommand(() -> vision.setCam(false)),
                new SequentialCommandGroup(
                        new GoToPointWithDefaultCommand(() -> SampleMovementOptimizer.getIntermediatePoint(storage.getCoarsePosition(), -15.0, 40, 25), gtpc),
                        new GoToPointWithDefaultCommand(() -> SampleMovementOptimizer.getClosestPoint(storage.getCoarsePosition(), -15.0, 40, 25), gtpc)

                        /*new RunIfCommand(
                                new InterruptCommand(new GoToPointWithDefaultCommand(
                                        () -> {
                                            Translation2d firstToSecond = storage.getCoarsePosition().minus(new Pose2d(-15.0, 40.0, new Rotation2d())).getTranslation();
                                            double angle = Math.atan2(firstToSecond.getY(), firstToSecond.getX());
                                            return new Pose2d(-15.0, 40.0, new Rotation2d(angle));
                                        }, gtpc),
                                        () -> pinpoint.getPose().getX() > -24.0),
                                // go here only if blocking
                                () -> {
                                    Translation2d firstTrans = new Pose2d(-15.0, 40.0, Rotation2d.fromDegrees(-60)).minus(AutoConstants.cycleScorePos).getTranslation();
                                    double angleToFirst = Math.atan2(firstTrans.getY(), firstTrans.getX());
                                    Translation2d secondTrans = storage.getCoarsePosition().minus(AutoConstants.cycleScorePos).getTranslation();
                                    double angleToSecond = Math.atan2(secondTrans.getY(), secondTrans.getX());
                                    return Math.abs(angleToSecond) > Math.abs(angleToFirst);
                                }),
                        new GoToPointWithDefaultCommand(storage::getCoarsePositionOffset, gtpc),*/
                ).alongWith(
                        new ParallelCommandGroup(
                            new InterruptCommand(new ExtendCommand(extension, 0.0), () -> extension.getCurrentInches() < 15.0),
                                new WristCommand(wrist, IntakeConstants.groundPos)
                        ).andThen(
                                new InterruptCommand(new PivotCommand(pivot, PivotConstants.bottomLimit), () -> pivot.getCurrentPosition() < 45.0),
                                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 0),
                                new IVKCommand(20.0, IVKCommand.intakeReadyY, extension, pivot).alongWith(
                                        new TurretCommand(turret, 0.0)
                                ).alongWith(
                                        // flip down wrist to a ready position
                                        new WaitCommand(400).andThen(new WristCommand(wrist, IntakeConstants.toptakePos))
                                )
                        )
                ),

                //new WristCommand(wrist, IntakeConstants.toptakePos - 0.1),
                //new FineAlignCommand(vision, gtpc, drive, pinpoint, turret).withTimeout(1500),
                //new WristCommand(wrist, IntakeConstants.toptakePos),
                //new WaitCommand(300),
                new TimeoutCommand(new StoreFinePositionCommand(vision, storage, pinpoint, pivot, extension, turret), 200),
                new GoToPointWithDefaultCommand(storage::getFinePosition, gtpc, 0.5, 2),

                new TimeoutCommand(
                        new SubPosCommand(extension, wrist, intake, pivot, 20.0), 300
                ),
                new WaitCommand(400),
                new SequentialCommandGroup(
                        new InterruptCommand(new GoToPointWithDefaultCommand(new Pose2d(-10.0, 48.0, Rotation2d.fromDegrees(-20)), gtpc),
                                () -> pinpoint.getPose().getY() > 30.0),
                        new GoToPointWithDefaultCommand(AutoConstants.cycleScorePos, gtpc)
                ).alongWith(
                        new InterruptCommand(
                                new RetractCommand(wrist, pivot, extension, turret, intake),
                                () -> pinpoint.getPose().minus(AutoConstants.cycleScorePos).getTranslation().getNorm() < 64.0
                        ).andThen(
                                new BucketPosCommand(extension, pivot, wrist, turret)
                        ),
                        new TimeoutCommand(new StoreCoarsePositionCommand(vision, storage, pinpoint), 500)
                ),
                new IntakeClawCommand(intake, IntakeConstants.openPos),
                new WaitCommand(100)
        );

    }

}
