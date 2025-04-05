package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.commands.custom.PreaimCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.InterruptCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ReloadCommand;
import org.firstinspires.ftc.teamcode.commands.custom.StoreCoarsePositionCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TimeoutCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IVKConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.lib.SamplePoseStorage;
import org.firstinspires.ftc.teamcode.lib.path.spline.CubicBezier;
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

    public AutoSubCycle(VisionSubsystem vision, PivotSubsystem pivot, ExtensionSubsystem extension, SamplePoseStorage storage, DefaultDualMoveCommand dmc, MecanumDriveSubsystem drive, PinpointSubsystem pinpoint, IntakeSubsystem intake, WristSubsystem wrist, TurretSubsystem turret, TargetingSubsystem target) {

        super(
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1),
                dmc.setGVF(),
                new GVFWithDefaultCommand(dmc.getGvfc(), new CubicBezier(AutoConstants.cycleScorePos.getX(), AutoConstants.cycleScorePos.getY(),
                        -40, 48,
                        -8, 40,
                        -8, 18)).alongWith(new RetractCommand(wrist, pivot, extension, turret, intake)),
                new ReloadCommand(target),
                dmc.setP2P(),
                new PreaimCommand(dmc.getGtpc(), pivot, extension, intake, wrist, turret, target, pinpoint),
                new TimeoutCommand(
                        new SubPosCommand(extension, wrist, intake, pivot, target.getIVKY(), IVKConstants.intakeY), 700
                ),
                new WaitCommand(400),
                dmc.setGVF(),
                new GVFWithDefaultCommand(dmc.getGvfc(), new CubicBezier(-8, 18,
                        -8, 30,
                        -40, 48,
                        AutoConstants.cycleScorePos.getX(), AutoConstants.cycleScorePos.getY())
                ).alongWith(
                        new InterruptCommand(
                                new RetractCommand(wrist, pivot, extension, turret, intake),
                                () -> pinpoint.getPose().minus(AutoConstants.cycleScorePos).getTranslation().getNorm() < 64.0
                        ).andThen(
                                new BucketPosCommand(extension, pivot, wrist, turret)
                        ),
                        new TimeoutCommand(new StoreCoarsePositionCommand(vision, storage, pinpoint), 500)
                ),
                dmc.setP2P(),
                new IntakeClawCommand(intake, IntakeConstants.openPos),
                new WaitCommand(100)
        );

    }


    public AutoSubCycle(VisionSubsystem vision, PivotSubsystem pivot, ExtensionSubsystem extension, SamplePoseStorage storage, DefaultGoToPointCommand gtpc, MecanumDriveSubsystem drive, PinpointSubsystem pinpoint, IntakeSubsystem intake, WristSubsystem wrist, TurretSubsystem turret, TargetingSubsystem target) {

        super(
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1),
                new GoToPointWithDefaultCommand(new Pose2d(-12, 48, Rotation2d.fromDegrees(-90)), gtpc)
                        .alongWith(new RetractCommand(wrist, pivot, extension, turret, intake)),
                new GoToPointWithDefaultCommand(new Pose2d(-8, 18, Rotation2d.fromDegrees(-90)), gtpc),
                new ReloadCommand(target),
                new PreaimCommand(gtpc, pivot, extension, intake, wrist, turret, target, pinpoint),
                new TimeoutCommand(
                        new SubPosCommand(extension, wrist, intake, pivot, target.getIVKY(), IVKConstants.intakeY), 700
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
