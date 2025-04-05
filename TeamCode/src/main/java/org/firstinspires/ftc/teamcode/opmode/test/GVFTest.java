package org.firstinspires.ftc.teamcode.opmode.test;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.group.DefaultGVFCommand;
import org.firstinspires.ftc.teamcode.commands.group.GVFWithDefaultCommand;
import org.firstinspires.ftc.teamcode.lib.path.follower.GVFFollower;
import org.firstinspires.ftc.teamcode.lib.path.spline.CubicBezier;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(group="test")
public class GVFTest extends Robot {

    @Override
    public void runOpMode() {
        initialize();
        pinpoint.reset();
        pinpoint.resetYaw();

        waitForStart();

        DefaultGVFCommand gvfc = new DefaultGVFCommand(mecanum, pinpoint, new CubicBezier(0, 0,
                10, 0, 2, 8, 14, 8));

        cs.schedule(gvfc);

        cs.schedule(
                new GVFWithDefaultCommand(gvfc, new CubicBezier(0, 0,
                        10, -10, 50, -20, 50, -30)).whenFinished(
                        () -> telemetry.addData("gvfc", "yayyyyy")
                )
        );

        while (opModeIsActive()) {
            update();
        }

        end();

    }


}
