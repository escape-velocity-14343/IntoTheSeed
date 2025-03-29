package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.commands.custom.CoarseAlignCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.FineAlignCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.InterruptCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TimeoutCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AutoSubCycle extends SequentialCommandGroup {

    public AutoSubCycle(VisionSubsystem vision, PivotSubsystem pivot, ExtensionSubsystem extension, DefaultGoToPointCommand gtpc, MecanumDriveSubsystem drive, PinpointSubsystem pinpoint, IntakeSubsystem intake, WristSubsystem wrist, TurretSubsystem turret) {

        super(
                new InstantCommand(() -> vision.setCam(true)),
                new SequentialCommandGroup(
                        new InterruptCommand(new GoToPointWithDefaultCommand(new Pose2d(-15.0, 40.0, Rotation2d.fromDegrees(-60)), gtpc),
                                () -> pinpoint.getPose().getX() > -24.0),
                        new InterruptCommand(new GoToPointWithDefaultCommand(new Pose2d(-15.0, 35.0, Rotation2d.fromDegrees(-90)), gtpc),
                                () -> true),

                        new TimeoutCommand(
                                new CoarseAlignCommand(gtpc, vision, pinpoint), 2000
                        ),
                        new InstantCommand(() -> vision.setCam(false))
                ).alongWith(
                        new SubPosReadyCommand(extension, pivot, wrist, intake, turret, () -> 0, SlideConstants.submersibleIntakeMaxExtension, () -> true)
                ),
                new WristCommand(wrist, IntakeConstants.toptakePos - 0.1),
                new FineAlignCommand(vision, gtpc, drive, pinpoint, turret).withTimeout(1500),
                new WristCommand(wrist, IntakeConstants.toptakePos),
                new WaitCommand(300),

                new TimeoutCommand(
                        new SubPosCommand(extension, wrist, intake, pivot, SlideConstants.submersibleIntakeMaxExtension), 300
                ),
                new WaitCommand(500),
                new SequentialCommandGroup(
                        new InterruptCommand(new GoToPointWithDefaultCommand(new Pose2d(-10.0, 48.0, Rotation2d.fromDegrees(-20)), gtpc),
                                () -> pinpoint.getPose().getY() > 30.0),
                        new GoToPointWithDefaultCommand(AutoConstants.scorePos, gtpc)
                ).alongWith(
                        new RetractCommand(wrist, pivot, extension, turret, intake).andThen(
                                new BucketPosCommand(extension, pivot, wrist, turret)
                        )
                ),
                new IntakeClawCommand(intake, IntakeConstants.openPos),
                new WaitCommand(100)
        );

    }

}
