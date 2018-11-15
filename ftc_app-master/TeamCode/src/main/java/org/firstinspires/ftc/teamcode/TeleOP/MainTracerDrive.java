package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

@TeleOp(name="Tracer Drive", group="Drive")
public class MainTracerDrive extends OpMode {

    Robot robot = new Robot();

    @Override
    public void init() {

        robot.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {
    robot.lift.manualControl(gamepad2);
    robot.driveTrain.manualDrive(gamepad1);
    robot.acc.manualControl(gamepad2);
    robot.voice.manualControl(gamepad1);
    }
}
