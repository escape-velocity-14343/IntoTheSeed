package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.lib.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@Config
@TeleOp(group="Test")
public class ExtensionPIDTest extends LinearOpMode {
    public PivotSubsystem pivot;
    public ExtensionSubsystem extension;
    public WristSubsystem wrist;
    public static double targetInches = 0;
    public static double targetDegrees = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        CachingVoltageSensor voltage = new CachingVoltageSensor(hardwareMap);
        pivot = new PivotSubsystem(hardwareMap, voltage);
        extension = new ExtensionSubsystem(hardwareMap, pivot, voltage);
        wrist = new WristSubsystem(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        CommandScheduler.getInstance().schedule(new InstantCommand(() -> extension.setManualControl(false)));
        while (!isStopRequested()) {
            CommandScheduler.getInstance().schedule(new ExtendCommand(extension, targetInches));
            CommandScheduler.getInstance().schedule((new PivotCommand(pivot, targetDegrees)));
            CommandScheduler.getInstance().schedule(new WristCommand(wrist, IntakeConstants.foldedPos));

            telemetry.addData("current pos", extension.getCurrentPosition());
            telemetry.addData("target", targetInches);
            telemetry.addData("current inches", extension.getCurrentInches());
            telemetry.addData("is there", extension.isClose(targetInches));
            telemetry.update();

            CommandScheduler.getInstance().run();
        }
        CommandScheduler.getInstance().reset();
    }
}
