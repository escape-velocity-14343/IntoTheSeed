package org.firstinspires.ftc.teamcode.opmode.test;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.custom.PreaimCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ReloadCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class HoverTest extends Robot {
    DefaultGoToPointCommand gtpc;

    @Override
    public void runOpMode() throws InterruptedException {
        AutoConstants.alliance = AutoConstants.Alliance.BLUE;

        initialize();
        pinpoint.reset();

        wrist.setWrist(IntakeConstants.scoringPosReversed);
        intake.setClawer(IntakeConstants.closedPos);

        vision.waitForSetExposure(3000, 10000, PNPTest.exposure);
        while (!vision.setCam(true)) ;

        waitForStart();

        imu.resetYaw();
        extension.reset();
        gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d(-8, 18.5, Rotation2d.fromDegrees(-90)));
        pivot.setTarget(PivotConstants.topLimit);

        pinpoint.setHeading(-8, 18.5, -90);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ReloadCommand(target),
                        new PreaimCommand(gtpc, pivot, extension, intake, wrist, turret, target, pinpoint)
        ));

        cs.schedule(gtpc);

        while (opModeIsActive()) {
            update();
            telemetry.addData("pose", pinpoint.getPose());
        }

        end();
    }
}
