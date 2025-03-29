package org.firstinspires.ftc.teamcode.opmode.test;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.custom.CoarseAlignCommand;
import org.firstinspires.ftc.teamcode.commands.custom.FineAlignCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosCommand;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp(group = "test")
public class FineAlignTest extends Robot {

    @Override
    public void runOpMode() {
        initialize();
        VisionSubsystem vision = new VisionSubsystem(hardwareMap, telemetry);
        vision.waitForSetExposure(3000, 10000, PNPTest.exposure);
        while (!vision.setCam(false));

        waitForStart();
        pinpoint.setPosition(0, 0);
        pinpoint.resetYaw();

        DefaultGoToPointCommand gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, new Pose2d());


        cs.schedule(gtpc);

        cs.schedule(
                new SequentialCommandGroup(
                        retract(),
                        //new CoarseAlignCommand(gtpc, vision, pinpoint),
                        intakeReady(),
                        //new InstantCommand(()->vision.setCam(false)),
                        new WristCommand(wrist, IntakeConstants.toptakePos - 0.1),
                        new FineAlignCommand(vision, gtpc, mecanum, pinpoint, turret),
                        new WristCommand(wrist, IntakeConstants.toptakePos),
                        new SubPosCommand(extension, wrist, intake, pivot, SlideConstants.submersibleIntakeMaxExtension)
                )
        );

        while (opModeIsActive()) {
            update();
        }

        end();
    }

}
