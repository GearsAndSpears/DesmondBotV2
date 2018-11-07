package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Accumulator;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@TeleOp(name="Encoder Test", group="Tests")
public class EncoderTest extends OpMode {

    Robot robot = new Robot();

    @Override
    public void init() {

        robot.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {

        telemetry.addData("Acc Position:", robot.acc.accDrive.getCurrentPosition());
        telemetry.update();

        if(getRuntime() < 5){
            robot.acc.setArmState(Accumulator.accDrivePosition.DEPLOYED);
        }
        else{
            robot.acc.setArmState(Accumulator.accDrivePosition.RETRACTED);
        }

    }
}
