package org.firstinspires.ftc.teamcode.opmode.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IVKCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.InterruptCommand;
import org.firstinspires.ftc.teamcode.commands.custom.StoreCoarsePositionCommand;
import org.firstinspires.ftc.teamcode.commands.custom.StoreFinePositionCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TimeoutCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.BucketPos2Command;
import org.firstinspires.ftc.teamcode.commands.group.BucketPosCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultDualMoveCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGVFCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GVFWithDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.group.RetractCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.SamplePoseStorage;
import org.firstinspires.ftc.teamcode.lib.path.spline.CubicBezier;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(group = "test")
public class DepositTest extends Robot {
    DefaultGoToPointCommand gtpc;
    DefaultGVFCommand gvfc;
    DefaultDualMoveCommand dmc;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        pinpoint.reset();
        pinpoint.setPosition(0, 24.0);

        DefaultGoToPointCommand gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d(0.0, -24.0, Rotation2d.fromDegrees(-90.0)));

        gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d(-65, 40, new Rotation2d()));
        gvfc = new DefaultGVFCommand(mecanum, pinpoint, new CubicBezier(0, 0, 0, 0, 0, 0, 0, 0));

        dmc = new DefaultDualMoveCommand(mecanum, pinpoint, gtpc, gvfc);
        dmc.setState(DefaultDualMoveCommand.MoveState.P2P);

        cs.schedule(gtpc);

        cs.schedule(
                new SequentialCommandGroup(
                        dmc.setGVF(),
                        new InterruptCommand(
                                new GVFWithDefaultCommand(dmc.getGvfc(), 5, 10, () -> new CubicBezier[]{new CubicBezier(
                                        pinpoint.getPose().getX(), pinpoint.getPose().getY(),
                                        -30 - 0.5 * pinpoint.getPose().getX() + 10, 35,
                                        -52, 48,
                                        AutoConstants.scorePos.getX(), AutoConstants.scorePos.getY()
                                )}
                                ).reverseHeading(),
                                () -> AutoConstants.scorePos.minus(pinpoint.getPose()).getTranslation().getNorm() < 5.0
                        ).alongWith(
                                new InterruptCommand(
                                        new RetractCommand(wrist, pivot, extension, turret, intake),
                                        () -> pinpoint.getPose().getY() - extension.getCurrentInches() > 16
                                ).andThen(
                                        new BucketPos2Command(extension, pivot, wrist, turret, false)
                                ),
                                new WaitUntilCommand(() -> AutoConstants.scorePos.minus(pinpoint.getPose()).getTranslation().getNorm() < AutoConstants.fastDropDistance).andThen(
                                        new IntakeControlCommand(intake, IntakeConstants.openPos, 0.5)
                                )
                        )
                )
        );

        while (opModeIsActive()) {
            update();
        }

        end();
    }

}
