package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.custom.BucketRelocalizeCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IVKCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.InterruptCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SlowExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TimeoutCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.custom.VoltagePause;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.AutoSubCycle;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.BucketToIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultDualMoveCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGVFCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.DunkCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakePosCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.SamplePoseStorage;
import org.firstinspires.ftc.teamcode.lib.path.spline.CubicBezier;
import org.firstinspires.ftc.teamcode.opmode.test.PNPTest;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.Set;

@Autonomous(name = "Ezell's 8 Sample")
public class EZ7Piece extends Robot {
    DefaultGoToPointCommand gtpc;
    DefaultGVFCommand gvfc;
    DefaultDualMoveCommand dmc;

    @Override
    public void runOpMode() {
        AutoConstants.alliance = AutoConstants.Alliance.BLUE;

        initialize();
        pinpoint.reset();
        cs.registerSubsystem();

        wrist.setWrist(IntakeConstants.scoringPosReversed);
        intake.setClawer(IntakeConstants.closedPos);

        SamplePoseStorage storage = new SamplePoseStorage();
        storage.setCoarsePosition(new Pose2d(-9, 2, new Rotation2d()));
        vision.waitForSetExposure(3000, 10000, PNPTest.exposure);
        while (!vision.setCam(true)) ;

        waitForStart();

        imu.resetYaw();
        extension.reset();
        gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d(-65, 40, new Rotation2d()));
        gvfc = new DefaultGVFCommand(mecanum, pinpoint, new CubicBezier(0, 0, 0, 0, 0, 0, 0, 0));

        dmc = new DefaultDualMoveCommand(mecanum, pinpoint, gtpc, gvfc);
        dmc.setState(DefaultDualMoveCommand.MoveState.P2P);

        pivot.setTarget(PivotConstants.topLimit);

        pinpoint.setPosition(-65, 40);


        cs.schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> SlideConstants.lowExtend = true),
                        // score preload
                        new GoToPointWithDefaultCommand(AutoConstants.scorePos.plus(new Transform2d(new Translation2d(1,-1), new Rotation2d())), gtpc, 500, 100).alongWith(
                                new DunkCommand(extension, pivot, wrist, turret, intake)
                        ),

                        // intake first
                        new GoToPointWithDefaultCommand(
                                new Pose2d(-46, 47.5, new Rotation2d()), gtpc)
                                .alongWith(
                                        new ExtendCommand(extension, AutoConstants.spike1ExtensionLength - AutoConstants.extendedWhilePivotOffset - 2)
                                                .alongWith(
                                                        new WaitUntilCommand(() -> extension.getCurrentInches() < SlideConstants.pivotDownExtension).andThen(
                                                                new PivotCommand(pivot, 0)
                                                        )
                                                ),
                                        new WristCommand(wrist, IntakeConstants.groundPos),
                                        new TurretCommand(turret, 0),
                                        new IntakeControlCommand(intake, (2 * IntakeConstants.openPos + IntakeConstants.singleIntakePos) / 3, 1)
                                ),
                        //new WaitUntilCommand(() -> pivot.getPivotVelocity() < PivotConstants.maxPivotVelocity),
                        new ParallelCommandGroup(
                                new SlowExtendCommand(extension, AutoConstants.spike1ExtensionLength, AutoConstants.spikeExtensionSpeed),
                                new WaitUntilCommand(() -> (extension.getCurrentInches() > AutoConstants.spike1ExtensionLength - AutoConstants.clawCloseDistance) || intake.proxClose()
                                ).andThen(
                                        new IntakeControlCommand(intake, IntakeConstants.closedPos, 1)
                                )
                        ),
                        new GoToPointWithDefaultCommand(AutoConstants.scorePos, gtpc, 500, 100).alongWith(
                                new DunkCommand(extension, pivot, wrist, turret, intake)
                        ),

                        // intake second
                        new GoToPointWithDefaultCommand(
                                new Pose2d(-46, 57.5, new Rotation2d()), gtpc).alongWith(
                                new ExtendCommand(extension, AutoConstants.spike2ExtensionLength - AutoConstants.extendedWhilePivotOffset)
                                        .alongWith(
                                                new WaitUntilCommand(() -> extension.getCurrentInches() < SlideConstants.pivotDownExtension).andThen(
                                                        new PivotCommand(pivot, 0)
                                                )
                                        ),
                                new WristCommand(wrist, IntakeConstants.groundPos),
                                new TurretCommand(turret, 0),
                                new IntakeControlCommand(intake, (2 * IntakeConstants.openPos + IntakeConstants.singleIntakePos) / 3, 1)
                        ),
                        new ParallelCommandGroup(
                                new SlowExtendCommand(extension, AutoConstants.spike2ExtensionLength, AutoConstants.spikeExtensionSpeed),
                                new WaitUntilCommand(() -> (extension.getCurrentInches() > AutoConstants.spike2ExtensionLength - AutoConstants.clawCloseDistance) || intake.proxClose()
                                ).andThen(
                                        new IntakeControlCommand(intake, IntakeConstants.closedPos, 1)
                                )
                        ),
                        new GoToPointWithDefaultCommand(AutoConstants.scorePos, gtpc, 100, 100).alongWith(
                                new DunkCommand(extension, pivot, wrist, turret, intake)
                        ),


                        // intake third
                        // The thing is that you have to make sure you don't slam your intake on the ground and pop it
                        new GoToPointWithDefaultCommand(new Pose2d(-45, 60, Rotation2d.fromDegrees(10)), gtpc, 5, 5)
                                .withTimeout(1500)
                                .alongWith(
                                        new ExtendCommand(extension, AutoConstants.spike3ExtensionLength - AutoConstants.extendedWhilePivotOffset)
                                                .alongWith(
                                                        new WaitUntilCommand(() -> extension.getCurrentInches() < SlideConstants.pivotDownExtension).andThen(
                                                                new PivotCommand(pivot, 0)
                                                        )
                                                ),
                                        new TurretCommand(turret, 0),
                                        new WristCommand(wrist, IntakeConstants.groundPos),
                                        new IntakeControlCommand(intake, (2 * IntakeConstants.openPos + IntakeConstants.singleIntakePos) / 3, 1)
                                ),
                        new GoToPointWithDefaultCommand(
                                new Pose2d(-44, 60, Rotation2d.fromDegrees(24)), gtpc, 3, 3
                        ),
                        new ParallelCommandGroup(
                                new TimeoutCommand(new SlowExtendCommand(extension, AutoConstants.spike3ExtensionLength, AutoConstants.spikeExtensionSpeed), 2000),
                                new WaitUntilCommand(() -> (extension.getCurrentInches() > AutoConstants.spike3ExtensionLength - AutoConstants.clawCloseDistance) || intake.proxClose()
                                ).andThen(
                                        new IntakeControlCommand(intake, IntakeConstants.closedPos, 1)
                                )
                        ),
                        new GoToPointWithDefaultCommand(AutoConstants.scorePos, gtpc, 500, 100).alongWith(
                                new DunkCommand(extension, pivot, wrist, turret, intake)
                        ),
                        //new TimeoutCommand(new WristCommand(wrist, IntakeConstants.groundPos), 1),
                        new AutoSubCycle(vision, pivot, extension, storage, dmc, mecanum, pinpoint, intake, wrist, turret, target, basketSensor),
                        new AutoSubCycle(vision, pivot, extension, storage, dmc, mecanum, pinpoint, intake, wrist, turret, target, basketSensor),
                        new AutoSubCycle(vision, pivot, extension, storage, dmc, mecanum, pinpoint, intake, wrist, turret, target, basketSensor),
                        new AutoSubCycle(vision, pivot, extension, storage, dmc, mecanum, pinpoint, intake, wrist, turret, target, basketSensor)
                )
        );

        cs.schedule(dmc);

        while (opModeIsActive()) {
            update();
        }

        SlideConstants.lowExtend = false;
        turret.rotateTo(0);
        intake.setClawer(IntakeConstants.singleIntakePos);
        end();
    }
}
