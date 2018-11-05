package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

import static org.firstinspires.ftc.teamcode.Hardware.Accumulator.accDrivePosition;

@Autonomous(name="FacingCrater V2", group="Pushbot")

public class FacingCrater extends  BasicAuto{

    public Robot robot = new Robot();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        robot.acc.setArmState(accDrivePosition.RETRACTED);

    }
}
