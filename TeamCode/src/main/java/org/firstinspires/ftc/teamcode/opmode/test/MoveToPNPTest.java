package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.custom.CoarseAlignCommand;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.lib.RobotPnP;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.concurrent.atomic.AtomicReference;

@Config
@TeleOp(group = "test")
public class MoveToPNPTest extends Robot {

    public static boolean red = true;
    VisionSubsystem vision;

    public static double cx = 338.083;
    public static double cy = 218.771;
    public static double focalL = 491.437;

    public enum VisionState {
        FIND,
        MOVE
    }

    VisionState currentState = VisionState.FIND;
    RobotPnP pnp;

    public static int exposure = 40;
    public static boolean botStream = true;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        pnp = new RobotPnP(cx, cy, focalL);

        if (red) {
            AutoConstants.alliance = AutoConstants.Alliance.RED;
        } else {
            AutoConstants.alliance = AutoConstants.Alliance.BLUE;
        }

        // vision = new VisionSubsystem(hardwareMap, telemetry);

        while (vision.setCam(false));

        vision.waitForSetExposure(3000, 10000, exposure);

        while (vision.setCam(true));
        vision.waitForSetExposure(3000, 10000, exposure);

        CommandScheduler.getInstance().registerSubsystem(vision);

        while (opModeInInit()) {
            vision.setCam(botStream);
        }

        waitForStart();

        AtomicReference<Translation2d> sampleFCPos = new AtomicReference<>();

        DefaultGoToPointCommand gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, pinpoint.getPose());

        ElapsedTime timer = new ElapsedTime();
        boolean thing = false;

        cs.schedule(
                gtpc
        );

        cs.schedule(
                new SequentialCommandGroup(
                        new WaitCommand(150),
                        new CoarseAlignCommand(gtpc, vision, pinpoint).alongWith(
                                topIntakeReady(() -> 0)
                        )
                        /*(new InstantCommand(() -> {
                            Vector2d samplePos = cameraSubsystem.getSamplePos();
                            sampleFCPos.set(pnp.getFieldCoordinates((int) samplePos.getX(), (int) samplePos.getY(), pinpoint.getPose()));
                        })*/)

        );

        while (opModeIsActive()) {


            update();
            /*if (timer.seconds() > 1 && !thing) {
                thing = true;
                cs.schedule(
                        new GoToPointWithDefaultCommand(
                                new Pose2d(
                                        sampleFCPos.get().getX() - 24,
                                        sampleFCPos.get().getY(),
                                        new Rotation2d()
                                ), gtpc
                        ).alongWith(
                                intakeReady(() -> 0)
                        )
                );
                Log.i("PNPTest", "Sample x:" + sampleFCPos.get().getX());
                Log.i("PNPTest", "Sample y:" + sampleFCPos.get().getY());
            }*/
        }
    }

}

