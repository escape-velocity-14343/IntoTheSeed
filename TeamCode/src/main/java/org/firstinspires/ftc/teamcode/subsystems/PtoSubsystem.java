package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;

public class PtoSubsystem extends SubsystemBase {
    private Servo leftServo, rightServo;
    private boolean engaged;
    public PtoSubsystem(HardwareMap hardwareMap) {
        leftServo = hardwareMap.servo.get("leftPTO");
        rightServo = hardwareMap.servo.get("rightPTO");
        //leftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        //rightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }
    public void setEngaged(boolean engage) {
        engaged = engage;
        if(engage) {
            leftServo.setPosition(DriveConstants.ptoLeftEngagedPos);
            rightServo.setPosition(DriveConstants.ptoRightEngagedPos);
        }
        else {
            leftServo.setPosition(DriveConstants.ptoLeftDisengagedPos);
            rightServo.setPosition(DriveConstants.ptoRightDisengagedPos);
        }
    }

    public boolean isEngaged() {
        return engaged;
    }
}


