package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;

@Autonomous(name = "BLUE Ezell's 7 Sample")
public class EZ7PieceBlue extends EZ7Piece {

    @Override
    public void runOpMode() {
        AutoConstants.alliance = AutoConstants.Alliance.BLUE;
        super.runOpMode();
    }

}
