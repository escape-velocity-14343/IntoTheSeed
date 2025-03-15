package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
public class SampleTest extends LinearOpMode {
    public static double config = 0;

    CommandScheduler cs = CommandScheduler.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (!isStopRequested()){
        }

        cs.reset();
    }
}
