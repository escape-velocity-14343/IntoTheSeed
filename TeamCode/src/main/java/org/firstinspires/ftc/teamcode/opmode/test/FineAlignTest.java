package org.firstinspires.ftc.teamcode.opmode.test;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.custom.CoarseAlignCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.FineAlignCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IVKCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SequentialIVKCommand;
import org.firstinspires.ftc.teamcode.commands.custom.StoreFinePositionCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TimeoutCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosCommand;
import org.firstinspires.ftc.teamcode.constants.IVKConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.SamplePoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp(group = "test")
public class FineAlignTest extends Robot {

    @Override
    public void runOpMode() {
        initialize();
        //VisionSubsystem vision = new VisionSubsystem(hardwareMap, telemetry);
        SamplePoseStorage storage = new SamplePoseStorage();
        vision.waitForSetExposure(3000, 10000, PNPTest.exposure);
        while (!vision.setCam(false));

        waitForStart();
        pinpoint.setPosition(0, 0);
        pinpoint.resetYaw();

        DefaultGoToPointCommand gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d());


        cs.schedule(gtpc);

        cs.schedule(
                new SequentialCommandGroup(
                        retract().andThen(
                                new IntakeClawCommand(intake, IntakeConstants.openPos)
                        ),
                        //new CoarseAlignCommand(gtpc, vision, pinpoint),
                        new SequentialIVKCommand(SlideConstants.submersibleIntakeMidExtension, IVKCommand.intakeReadyY, extension, pivot).alongWith(
                                new TurretCommand(turret, 0.0),
                                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 0),
                                // flip down wrist to a ready position
                                new WaitUntilCommand(() -> extension.getCurrentInches() > 10.0).andThen(new WristCommand(wrist, IntakeConstants.toptakePos))
                        ),
                        //new InstantCommand(()->vision.setCam(false)),
                        new WristCommand(wrist, IntakeConstants.toptakePos - 0.1),
                        new StoreFinePositionCommand(vision, storage, pinpoint, pivot, extension, turret),
                        new GoToPointWithDefaultCommand(storage::getFinePosition, gtpc, 0.5, 2).alongWith(
                                new ExtendCommand(extension, storage::getNewExtension),
                                new WristCommand(wrist, IntakeConstants.toptakePos)
                        ),
                        /*
                        new StoreFinePositionCommand(vision, storage, pinpoint, pivot, extension, turret),
                        new GoToPointWithDefaultCommand(storage::getFinePosition, gtpc, 0.5, 2).alongWith(
                                new ExtendCommand(extension, storage::getNewExtension),

                        ),*/
                        new TimeoutCommand(
                                new SubPosCommand(extension, wrist, intake, pivot,
                                        () -> Math.cos(Math.toRadians(pivot.getCurrentPosition())) * extension.getCurrentInches()
                                ), 700
                        ).alongWith(
                                new InstantCommand(() ->
                                        Log.i("subpos", "forward: " + Math.cos(pivot.getCurrentPosition()) * extension.getCurrentInches()))
                        )
                        //new FineAlignCommand(vision, gtpc, mecanum, pinpoint, turret),
                        //new WristCommand(wrist, IntakeConstants.toptakePos),
                        //new SubPosCommand(extension, wrist, intake, pivot, SlideConstants.submersibleIntakeMaxExtension)
                )
        );

        while (opModeIsActive()) {
            update();
        }

        end();
    }

}
