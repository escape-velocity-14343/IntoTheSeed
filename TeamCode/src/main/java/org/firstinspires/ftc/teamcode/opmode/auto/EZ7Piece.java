package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
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
import org.firstinspires.ftc.teamcode.commands.custom.TimeoutCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.custom.VoltagePause;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.AutoSubCycle;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.BucketToIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.group.IntakePosCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.opmode.test.PNPTest;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(name = "Ezell's 6 Sample")
public class EZ7Piece extends Robot {

    DefaultGoToPointCommand gtpc;

    @Override
    public void runOpMode() {
        AutoConstants.alliance = AutoConstants.Alliance.BLUE;

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
                        new InstantCommand(() -> SlideConstants.lowExtend = true),
                        // score preload
                        new GoToPointWithDefaultCommand(AutoConstants.scorePos, gtpc).alongWith(
                                new IntakeClawCommand(intake, IntakeConstants.closedPos),
                                new BucketPosCommand(extension, pivot, wrist, turret)
                        ),
                        new VoltagePause(voltage, 1),
                        new IntakeClawCommand(intake, IntakeConstants.openPos),
                        new VoltagePause(voltage),

                        // intake first
                        new GoToPointWithDefaultCommand(
                                new Pose2d(-46, 47.5, new Rotation2d()), gtpc
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
                                new SlowExtendCommand(extension, AutoConstants.spikeExtensionLength, AutoConstants.spikeExtensionSpeed),
                                new WaitUntilCommand(() -> extension.getCurrentInches() > AutoConstants.spikeExtensionLength - 1
                                ).andThen(
                                        new IntakeControlCommand(intake, IntakeConstants.closedPos, 1)
                                )
                        ),
                        new GoToPointWithDefaultCommand(AutoConstants.scorePos, gtpc).alongWith(
                                new BucketPosCommand(extension, pivot, wrist, turret)
                        ),
                        new VoltagePause(voltage, 1),
                        new IntakeClawCommand(intake, IntakeConstants.openPos),
                        new VoltagePause(voltage),

                        // intake second
                        new GoToPointWithDefaultCommand(
                                new Pose2d(-46, 57.5, new Rotation2d()), gtpc
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
                        new ParallelCommandGroup(
                                new SlowExtendCommand(extension, AutoConstants.spikeExtensionLength, AutoConstants.spikeExtensionSpeed),
                                new WaitUntilCommand(() -> extension.getCurrentInches() > AutoConstants.spikeExtensionLength - 1
                                ).andThen(
                                        new IntakeControlCommand(intake, IntakeConstants.closedPos, 1)
                                )
                        ),
                        new GoToPointWithDefaultCommand(AutoConstants.scorePosOffset, gtpc).alongWith(
                                new BucketPosCommand(extension, pivot, wrist, turret)
                        ),
                        new VoltagePause(voltage, 1),
                        new IntakeClawCommand(intake, IntakeConstants.openPos),
                        new VoltagePause(voltage),
                        new WaitCommand(200), //Wait before turning so no more rotational force flicking gamepiece out of bucket
                        //Probably can remove ^ this wait once new intake with slam is added


                        // intake third
                        // The thing is that you have to make sure you don't slam your intake on the ground and pop it
                        new GoToPointWithDefaultCommand(
                                new Pose2d(-39, 56, Rotation2d.fromDegrees(43)), gtpc, 0.5, 2
                        ).alongWith(
                                new BucketToIntakeCommand(pivot, extension, intake, wrist, turret, 9, 45, -1)
                        ),
                        new WristCommand(wrist, IntakeConstants.toptakePos),
                        new TimeoutCommand(
                                new IVKCommand(10, IVKCommand.intakeReadyY, extension, pivot),
                                1000),
                        new WaitUntilCommand(() -> pivot.getPivotVelocity() < PivotConstants.maxPivotVelocity).deadlineWith(new WaitCommand(1500)),
                        new TimeoutCommand(
                                new IVKCommand(10, IVKCommand.intakeY+1, extension, pivot),
                                1000),
                        new TimeoutCommand(
                                new IVKCommand(10, IVKCommand.intakeY, extension, pivot),
                                1000),
                        new TimeoutCommand(
                                new IVKCommand(10, IVKCommand.intakeY-2, extension, pivot),
                                1000),
                        new WaitCommand(200),
                        new GoToPointWithDefaultCommand(AutoConstants.scorePosOffset, gtpc).alongWith(
                                new BucketPosCommand(extension, pivot, wrist, turret)
                        ),
                        new VoltagePause(voltage, 1),
                        new IntakeClawCommand(intake, IntakeConstants.openPos),
                        new VoltagePause(voltage),
                        new AutoSubCycle(vision, pivot, extension, gtpc, mecanum, pinpoint, intake, wrist, turret),
                        new AutoSubCycle(vision, pivot, extension, gtpc, mecanum, pinpoint, intake, wrist, turret),
                        new AutoSubCycle(vision, pivot, extension, gtpc, mecanum, pinpoint, intake, wrist, turret)
                )
        );

        cs.schedule(gtpc);

        while (opModeIsActive()) {
            update();
        }

        SlideConstants.lowExtend = false;
        end();
    }
}
