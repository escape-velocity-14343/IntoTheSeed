package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.group.SubPosReadyCommand;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
public class VisionTurretTest extends LinearOpMode {
    public static double config = 0;
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

        waitForStart();

        while (!isStopRequested()){
            //cs.schedule(new TurretCommand(turret));
        }

        cs.reset();
    }
}
