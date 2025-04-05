package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.TargetingSubsystem;

public class ReloadCommand extends CommandBase {
    private TargetingSubsystem target;
    ElapsedTime time = new ElapsedTime();

    public ReloadCommand(TargetingSubsystem target){
        this.target = target;
    }

    @Override
    public void initialize() {
        time.reset();
    }

    @Override
    public void execute() {
        target.cycle();
    }

    @Override
    public boolean isFinished() {
        return time.seconds() > 0.2;
    }
}
