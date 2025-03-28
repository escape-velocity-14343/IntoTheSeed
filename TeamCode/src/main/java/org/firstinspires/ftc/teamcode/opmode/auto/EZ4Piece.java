package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IVKCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.InterruptCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SlowExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WaitUntilStabilizedCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.AutoSubCycle;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.BucketToIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.opmode.test.PNPTest;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(name = "Ezell's 6 Sample")
public class EZ4Piece extends Robot {

    DefaultGoToPointCommand gtpc;

    @Override
    public void runOpMode() {

        initialize();
        pinpoint.reset();
        wrist.setWrist(IntakeConstants.foldedPos);
        intake.setClawer(IntakeConstants.closedPos);

        VisionSubsystem vision = new VisionSubsystem(hardwareMap, telemetry);
        vision.waitForSetExposure(3000, 10000, PNPTest.exposure);
        while (!vision.setCam(true)) ;

        waitForStart();

        imu.resetYaw();
        extension.reset();
        gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d(-65, 40, new Rotation2d()));
        pinpoint.setPosition(-65, 40);


        cs.schedule(
                new SequentialCommandGroup(
                        // score preload
                        new GoToPointWithDefaultCommand(AutoConstants.scorePos, gtpc).alongWith(
                                new IntakeClawCommand(intake, IntakeConstants.closedPos),
                                new BucketPosCommand(extension, pivot, wrist, turret)
                        ),
                        new WaitCommand(100),
                        new IntakeClawCommand(intake, IntakeConstants.openPos),
                        new WaitCommand(50),


                        // intake first
                        new GoToPointWithDefaultCommand(
                                new Pose2d(-43, 47.5, new Rotation2d()), gtpc
                        ).alongWith(
                                new ExtendCommand(extension, SlideConstants.minExtension + 1),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> extension.getCurrentInches() < 15),
                                        new PivotCommand(pivot, 0)
                                ),
                                new WristCommand(wrist, IntakeConstants.groundPos),
                                new TurretCommand(turret, 0),
                                new IntakeControlCommand(intake, IntakeConstants.openPos, 1)
                        ),
                        //new WaitUntilCommand(() -> pivot.getPivotVelocity() < PivotConstants.maxPivotVelocity),
                        new ParallelCommandGroup(
                                new SlowExtendCommand(extension, 6, 0.7),
                                new WaitCommand(100).andThen(
                                        new IntakeControlCommand(intake, IntakeConstants.closedPos, 1)
                                )
                        ),
                        new WaitCommand(100),
                        new GoToPointWithDefaultCommand(AutoConstants.scorePos, gtpc).alongWith(
                                new BucketPosCommand(extension, pivot, wrist, turret)
                        ),
                        new WaitCommand(100),
                        new IntakeClawCommand(intake, IntakeConstants.openPos),
                        new WaitCommand(50),

                        // intake second
                        new GoToPointWithDefaultCommand(
                                new Pose2d(-42.5, 57.5, new Rotation2d()), gtpc
                        ).alongWith(
                                new ExtendCommand(extension, SlideConstants.minExtension + 1),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> extension.getCurrentInches() < 15),
                                        new PivotCommand(pivot, 0)
                                ),
                                new WristCommand(wrist, IntakeConstants.groundPos),
                                new TurretCommand(turret, 0),
                                new IntakeControlCommand(intake, IntakeConstants.openPos, 1)
                        ),
                        //new WaitUntilCommand(() -> pivot.getPivotVelocity() < PivotConstants.maxPivotVelocity),
                        new ParallelCommandGroup(
                                new SlowExtendCommand(extension, 6, 0.7),
                                new WaitCommand(100).andThen(
                                        new IntakeControlCommand(intake, IntakeConstants.closedPos, 1)
                                )
                        ),
                        new WaitCommand(100),
                        new GoToPointWithDefaultCommand(AutoConstants.scorePos, gtpc).alongWith(
                                new BucketPosCommand(extension, pivot, wrist, turret)
                        ),
                        new WaitCommand(100),
                        new IntakeClawCommand(intake, IntakeConstants.openPos),
                        new WaitCommand(50),


                        // intake third
                        new GoToPointWithDefaultCommand(
                                new Pose2d(-42, 50, Rotation2d.fromDegrees(45)), gtpc, 0.5, 2
                        ).alongWith(
                                new InterruptCommand(
                                        new BucketToIntakeCommand(pivot, extension, intake, wrist, turret, 6, 45, 0),
                                        () -> pinpoint.getPose().getY() < 55
                                ),
                                new BucketToIntakeCommand(pivot, extension, intake, wrist, turret, 9, 45, -1)
                        ),
                        new WaitUntilCommand(() -> pivot.getPivotVelocity() < PivotConstants.maxPivotVelocity),
                        new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1),
                        new ParallelDeadlineGroup(
                                new WaitCommand(900),
                                new IVKCommand(15, IVKCommand.intakeY, extension, pivot)
                        ),
                        new GoToPointWithDefaultCommand(AutoConstants.scorePos, gtpc).alongWith(
                                new BucketPosCommand(extension, pivot, wrist, turret)
                        ),
                        new WaitCommand(100),
                        new IntakeClawCommand(intake, IntakeConstants.openPos),
                        new WaitCommand(100),
                        new AutoSubCycle(vision, pivot, extension, gtpc, mecanum, pinpoint, intake, wrist, turret),
                        new AutoSubCycle(vision, pivot, extension, gtpc, mecanum, pinpoint, intake, wrist, turret),
                        new AutoSubCycle(vision, pivot, extension, gtpc, mecanum, pinpoint, intake, wrist, turret)

                )
        );

        cs.schedule(gtpc);

        while (opModeIsActive()) {
            update();
        }

        end();

    }

}
