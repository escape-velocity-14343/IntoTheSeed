package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.lib.Util;

@Config
public class WristSubsystem extends SubsystemBase {
    private ServoImplEx wrist;
    private double rotation = IntakeConstants.groundPos;
    public static double debug = 0;
    private boolean pwmDisabled = false;

    public WristSubsystem(HardwareMap hardwareMap) {
        wrist = (ServoImplEx) hardwareMap.get(Servo.class, "wrist");
        wrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void setWrist(double rotation) {
//        setPwmDisabled(false);
        wrist.setPosition(rotation + IntakeConstants.wristOffset);
        this.rotation = rotation + IntakeConstants.wristOffset;
    }

    // TODO: if we use analog thing make it return actual position
    public double getPosition() {
        return wrist.getPosition();
    }

    public boolean isClose(double target) {
        return Util.inRange(target, getPosition(), SlideConstants.tolerance);
    }

    public boolean isPwmDisabled() {
        return pwmDisabled;
    }

    public void setPwmDisabled(boolean disabled) {
        this.pwmDisabled = disabled;
        if (disabled) {
            wrist.setPwmDisable();
        } else {
            wrist.setPwmEnable();
        }
    }
}
