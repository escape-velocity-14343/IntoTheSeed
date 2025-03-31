package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeControlCommand extends CommandBase {
    IntakeSubsystem intake;
    double position = 0;
    double speed = 0;
    ElapsedTime time = new ElapsedTime();

    /**
     * Positive is intake, negative is outtake
     *
     * @param intake
     * @param position
     */
    public IntakeControlCommand(IntakeSubsystem intake, double position, double speed) {
        addRequirements(intake);
        this.intake = intake;
        this.position = position;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        time.reset();
        intake.setClawer(position);
        intake.setIntakeSpeed(speed);
        Log.i("%8", "Intake pos: " + position);
    }

    @Override
    public boolean isFinished() {
        return time.seconds() > 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        Log.i("%8", "IntakeControl command ended at: " + position);
    }
}
