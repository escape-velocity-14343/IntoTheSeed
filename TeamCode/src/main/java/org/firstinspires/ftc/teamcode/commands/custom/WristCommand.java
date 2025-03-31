package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {
    WristSubsystem wrist;
    double target = 0;
    ElapsedTime timer = new ElapsedTime();
    //0.115sec/60° @ 6.0V
    //0.1 is 60
    //0.3 is 180
    //0.6 is 360
    //0.69s for 360 degrees
    //0.69/0.6s for each position unit
    double timeNeeded = 0.4; // seconds

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
        Log.i("%3", "Wrist set to " + target +" it took: " + timer.seconds());
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= timeNeeded;
    }
}
