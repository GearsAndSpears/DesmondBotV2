package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="FacingCrater V2", group="Pushbot")

public class FacingCrater extends  BasicAuto{

    @Override
    public void runOpMode() {
        gyroTurn(3.0, 3.0);
    }
}
