package org.firstinspires.ftc.teamcode.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor

class MecanumDrivebaseInKotlin : LinearOpMode() {
    lateinit var fl: DcMotor
    lateinit var fr: DcMotor
    lateinit var bl: DcMotor
    lateinit var br: DcMotor

    override fun runOpMode() {

        var fl = hardwareMap.dcMotor.get("fl")
        var fr = hardwareMap.dcMotor.get("fr")
        var bl = hardwareMap.dcMotor.get("bl")
        var br = hardwareMap.dcMotor.get("br")

        waitForStart()

        while (!isStopRequested) {
            var x = gamepad1.left_stick_x
            var y = -gamepad1.left_stick_y
            var rot = gamepad1.right_stick_x
            fl.power
        }
    }


}