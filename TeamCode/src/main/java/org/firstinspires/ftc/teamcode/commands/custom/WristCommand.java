package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {

    WristSubsystem wrist;
    double target = 0;
    ElapsedTime timer = new ElapsedTime();
    double timeNeeded = 0.5; // seconds

    public WristCommand(WristSubsystem wrist, double target) {
        this.wrist = wrist;
        this.target = target;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        timer.reset();
        timeNeeded = IntakeConstants.timeMultiplier * Math.abs(wrist.getPosition() - target);
        wrist.setWrist(target);
    }

    @Override
    public void end(boolean wasInterrupted) {
        Log.i("%3", "Wrist set to " + target);
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= timeNeeded;
    }
}
