package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Accumulator;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@TeleOp(name="Lift Test", group="Tests")
public class LiftTest extends OpMode {

    Robot robot = new Robot();

    @Override
    public void init() {

        robot.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {

    robot.lift.manualControl(gamepad1);

    }
}
