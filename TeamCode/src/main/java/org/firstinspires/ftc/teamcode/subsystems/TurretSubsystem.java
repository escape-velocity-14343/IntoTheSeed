package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

public class TurretSubsystem extends SubsystemBase {

    Servo turret;

    public TurretSubsystem(HardwareMap hwmap) {
        turret = hwmap.servo.get("turret");
    }

    public void rotateTo(double rot) {
        turret.setPosition(getPositionFromAngle(rot));
    }

    private double getPositionFromAngle(double angle) {
        double range = IntakeConstants.maxAngle - IntakeConstants.minAngle;
        double correctedAngle = angle - IntakeConstants.minAngle;
        return correctedAngle / range;
    }

}
