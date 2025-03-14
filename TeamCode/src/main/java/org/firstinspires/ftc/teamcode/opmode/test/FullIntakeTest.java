package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@Config
@TeleOp(group = "Test")
public class FullIntakeTest extends LinearOpMode {
    public WristSubsystem wrist;
    public IntakeSubsystem intakeSubsystem;
    public TurretSubsystem turret;
    public static double wristPos = 0;
    public static double speed = 0;
    public static double claw = 0;
    public static double turretDegrees = 0;

    @Override
    public void runOpMode() {

        waitForStart();
        wrist = new WristSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(wrist, intakeSubsystem, turret);

        while (!isStopRequested()) {
            wrist.setWrist(wristPos);
            intakeSubsystem.setIntakeSpeed(speed);
            intakeSubsystem.setClawer(claw);
            turret.rotateTo(turretDegrees);

            CommandScheduler.getInstance().run();
        }
        CommandScheduler.getInstance().reset();
    }
}
