package org.firstinspires.ftc.teamcode.commands.custom;

import static org.firstinspires.ftc.teamcode.constants.AutoConstants.autoscoreMaxVel;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

public class WaitUntilStabilizedCommand extends CommandBase {
    private PinpointSubsystem pinpointSubsystem;

    ElapsedTime timer = new ElapsedTime();
    Double timeout = 0.02;

    public WaitUntilStabilizedCommand(PinpointSubsystem pinpointSubsystem){
        this.pinpointSubsystem = pinpointSubsystem;
        timer.reset();
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    public Command setTimeout(Double timeout) {
        this.timeout = timeout;
        return this;
    }

    @Override
    public boolean isFinished() {
        return (pinpointSubsystem.getVelocity().getTranslation().getNorm() < autoscoreMaxVel && (timer.seconds() > timeout));
    }

    @Override
    public void end(boolean interrupted) {
        Log.i("%10", "Stabilized Command Finished in " + timer.seconds() + " seconds");
    }
}
