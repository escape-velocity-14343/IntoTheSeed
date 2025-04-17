package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;

@Autonomous(name = "RED Ezell's 7 Sample")
public class EZ7PieceRed extends EZ7Piece {

    @Override
    public void runOpMode() {
        AutoConstants.alliance = AutoConstants.Alliance.RED;
        super.runOpMode();
    }

}
