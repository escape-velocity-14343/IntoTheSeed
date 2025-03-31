package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeClawCommand extends CommandBase {
    IntakeSubsystem intake;
    double position = 0;
    ElapsedTime time = new ElapsedTime();

    /**
     * Positive is outtake, negative is intake
     *
     * @param intake
     * @param position
     */
    public IntakeClawCommand(IntakeSubsystem intake, double position) {
        addRequirements(intake);
        this.intake = intake;
        this.position = position;
    }

    @Override
    public void initialize() {
        time.reset();
        intake.setClawer(position);
        Log.i("%8", "Claw pos: " + position);
    }

    @Override
    public boolean isFinished() {
        return time.seconds() > 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        Log.i("%8", "Claw command ended at: " + position);
    }
}
