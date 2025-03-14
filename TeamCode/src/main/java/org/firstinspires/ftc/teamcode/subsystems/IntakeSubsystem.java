package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private CRServo intake;
    private Servo clawer;
    private AnalogInput front, back;
    private double speed = 0;
    private double clawPos = IntakeConstants.closedPos;

    private ElapsedTime lastResetTime;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.crservo.get("intake");
        clawer = hardwareMap.servo.get("clawer");
        back = hardwareMap.analogInput.get("clawSens1");
        front = hardwareMap.analogInput.get("clawSens2");
        clawer.setDirection(Servo.Direction.REVERSE);
//        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        lastResetTime = new ElapsedTime();
    }

    /**
     * Speeds from 1.0 to -1.0 Positive is outtake Negative is intake
     *
     * @param speed
     */
    public void setIntakeSpeed(double speed) {
        this.speed = speed;
        intake.setPower(speed);
    }

    public void setClawer(double value) {
        clawPos = value;
        clawer.setPosition(clawPos + IntakeConstants.clawOffset);
    }

    public double getFrontV() {
        return front.getVoltage();
    }

    public double getBackV() {
        return back.getVoltage();
    }

    public boolean getDSensorSupplier() {
        return (front.getVoltage() > IntakeConstants.intakeSensorVoltageThres);
    }

    @Override
    public void periodic() {
        // reset intake servos every 30 seconds so they don't suddently stop
        if (lastResetTime.seconds() > 25) {
            intake.setPower(0);
            lastResetTime.reset();
        } else {
            intake.setPower(speed);
        }

        clawer.setPosition(clawPos + IntakeConstants.clawOffset);
    }
}
