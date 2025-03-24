package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.BucketToIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "Ezell's 4 Sample")
public class EZ4Piece extends Robot {

    DefaultGoToPointCommand gtpc;

    @Override
    public void runOpMode() {

        initialize();
        pinpoint.reset();
        wrist.setWrist(IntakeConstants.foldedPos);
        intake.setClawer(IntakeConstants.closedPos);

        waitForStart();

        imu.resetYaw();
        extension.reset();
        gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d(-65, 40, new Rotation2d()));
        pinpoint.setPosition(-65, 40);

        cs.schedule(gtpc);

        cs.schedule(
                new SequentialCommandGroup(
                        // score preload
                        new GoToPointWithDefaultCommand(AutoConstants.scorePos, gtpc).alongWith(
                                new BucketPosCommand(extension, pivot, wrist, turret)
                        ),
                        new IntakeClawCommand(intake, IntakeConstants.openPos),
                        new WaitCommand(100),

                        // intake first
                        new GoToPointWithDefaultCommand(
                                new Pose2d(-50.5, 49, new Rotation2d()), gtpc
                        ).alongWith(
                                new BucketToIntakeCommand(pivot, extension, intake, wrist, turret, 18, 0)
                        ),
                        intake(18),
                        new WaitCommand(300),
                        new GoToPointWithDefaultCommand(AutoConstants.scorePos, gtpc).alongWith(
                                new BucketPosCommand(extension, pivot, wrist, turret)
                        ),
                        new IntakeClawCommand(intake, IntakeConstants.openPos),

                        // intake second
                        new GoToPointWithDefaultCommand(
                                new Pose2d(-50.5, 59, new Rotation2d()), gtpc
                        ).alongWith(
                                new BucketToIntakeCommand(pivot, extension, intake, wrist, turret, 18, 0)
                        ),
                        intake(18),
                        new WaitCommand(300),
                        new GoToPointWithDefaultCommand(AutoConstants.scorePos, gtpc).alongWith(
                                new BucketPosCommand(extension, pivot, wrist, turret)
                        ),
                        new IntakeClawCommand(intake, IntakeConstants.openPos),
                        new WaitCommand(100),

                        // intake third
                        new GoToPointWithDefaultCommand(
                                new Pose2d(-44.5, 51, Rotation2d.fromDegrees(45)), gtpc
                        ).alongWith(
                                new BucketToIntakeCommand(pivot, extension, intake, wrist, turret, 18, 45)
                        ),
                        intake(18),
                        new WaitCommand(300),
                        new GoToPointWithDefaultCommand(AutoConstants.scorePos, gtpc).alongWith(
                                new BucketPosCommand(extension, pivot, wrist, turret)
                        ),
                        new IntakeClawCommand(intake, IntakeConstants.openPos)

                )
        );

        while (opModeIsActive()) {
            update();
        }

        end();

    }

}
