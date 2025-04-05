package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.custom.IVKCommand;
import org.firstinspires.ftc.teamcode.lib.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
@Config
public class IVKTest extends LinearOpMode {
    public static double x = 0;
    public static double y = 0;

    public ExtensionSubsystem extension;
    public PivotSubsystem pivot;
    public CachingVoltageSensor voltage;

    CommandScheduler cs = CommandScheduler.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        voltage = new CachingVoltageSensor(hardwareMap);
        pivot = new PivotSubsystem(hardwareMap, voltage);
        extension = new ExtensionSubsystem(hardwareMap, pivot, voltage);
        pivot.setExtensionSupplier(extension::getCurrentInches);

        cs.registerSubsystem(pivot, extension);

        waitForStart();

        while (!isStopRequested()) {
            cs.schedule(new IVKCommand(() -> x, () -> y, extension, pivot, 1));
            cs.run();
        }

        cs.reset();
    }
}
