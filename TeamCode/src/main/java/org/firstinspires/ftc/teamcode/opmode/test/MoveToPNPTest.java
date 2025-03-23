package org.firstinspires.ftc.teamcode.opmode.test;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosReadyCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.RobotPnP;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.atomic.AtomicReference;

@TeleOp(group = "test")
public class MoveToPNPTest extends Robot {

    public static boolean red = true;
    VisionSubsystem highCameraSubsystem;
    VisionSubsystem slideCameraSubsystem;

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

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        pnp = new RobotPnP(cx, cy, focalL);

        if (red) {
            AutoConstants.alliance = AutoConstants.Alliance.RED;
        } else {
            AutoConstants.alliance = AutoConstants.Alliance.BLUE;
        }
        int[] viewportids = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);

        slideCameraSubsystem = new VisionSubsystem(hardwareMap, VisionConstants.slideCameraName, telemetry, viewportids[1]);
        highCameraSubsystem = new VisionSubsystem(hardwareMap, VisionConstants.chassisCameraName, telemetry, viewportids[0]);



        highCameraSubsystem.waitForSetExposure(3000, 10000, exposure);
        slideCameraSubsystem.waitForSetExposure(3000, 10000, exposure);

        CommandScheduler.getInstance().registerSubsystem(highCameraSubsystem, slideCameraSubsystem);

        waitForStart();

        AtomicReference<Vector2d> sampleFCPos = new AtomicReference<>();

        DefaultGoToPointCommand gtpc = new DefaultGoToPointCommand(mecanum, pinpoint, pinpoint.getPose());

        ElapsedTime timer = new ElapsedTime();
        boolean thing = false;

        cs.schedule(
                gtpc
        );

        cs.schedule(
                new SequentialCommandGroup(
                        new WaitCommand(150),

                        new InstantCommand(() -> {
                            Vector2d samplePos = highCameraSubsystem.getSamplePos();
                            sampleFCPos.set(pnp.getFieldCoordinates((int) samplePos.getX(), (int) samplePos.getY(), pinpoint.getPose()));
                        }))

        );

        while (opModeIsActive()) {
            update();
            if (timer.seconds() > 1 && !thing) {
                thing = true;
                cs.schedule(
                        new GoToPointWithDefaultCommand(
                                new Pose2d(
                                        sampleFCPos.get().getX() - 24,
                                        sampleFCPos.get().getY(),
                                        new Rotation2d()
                                ), gtpc
                        ).alongWith(
                                intakeReady(0)
                        )
                );
                Log.i("PNPTest", "Sample x:" + sampleFCPos.get().getX());
                Log.i("PNPTest", "Sample y:" + sampleFCPos.get().getY());
            }
        }
    }

}

