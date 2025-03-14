package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.lib.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@Config
@TeleOp(group = "Test")
public class PivotPIDTest extends LinearOpMode {
    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CachingVoltageSensor voltage = new CachingVoltageSensor(hardwareMap);
        PivotSubsystem pivot = new PivotSubsystem(hardwareMap, voltage);
        ExtensionSubsystem extension = new ExtensionSubsystem(hardwareMap, pivot, voltage);
        WristSubsystem wrist = new WristSubsystem(hardwareMap);
        pivot.setExtensionSupplier(extension::getCurrentInches);
        waitForStart();
        while (!isStopRequested()) {
            pivot.tiltToPos(target);
            wrist.setWrist(IntakeConstants.foldedPos);

            telemetry.addData("current pos", pivot.getCurrentPosition());
            telemetry.addData("target", target);
            telemetry.addData("is there", pivot.isClose(target));
            telemetry.update();

            CommandScheduler.getInstance().run();
        }
        CommandScheduler.getInstance().reset();
    }
}
