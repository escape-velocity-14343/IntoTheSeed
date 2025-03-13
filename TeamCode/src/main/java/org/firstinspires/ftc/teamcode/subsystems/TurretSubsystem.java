package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

public class TurretSubsystem extends SubsystemBase {

    Servo turret;
    private double position = 0;

    public TurretSubsystem(HardwareMap hwmap) {
        turret = hwmap.servo.get("turret");
    }

    public void rotateTo(double rot) {
        position = Range.clip(rot, IntakeConstants.minAngle, IntakeConstants.maxAngle);
        turret.setPosition(getPositionFromAngle(rot));
    }

    private double getPositionFromAngle(double angle) {
        double range = IntakeConstants.maxAngle - IntakeConstants.minAngle;
        double correctedAngle = angle - IntakeConstants.minAngle;
        return correctedAngle / range;
    }

    public double getPosition() {
        return position;
    }
}
