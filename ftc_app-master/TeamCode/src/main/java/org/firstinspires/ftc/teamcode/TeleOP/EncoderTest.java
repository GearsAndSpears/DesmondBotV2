package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Accumulator;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@TeleOp(name="Encoder Test", group="Tests")
public class EncoderTest extends LinearOpMode {

    Robot robot = new Robot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        robot.setup(hardwareMap, telemetry, this);

        robot.lift.liftLatch.setPosition(robot.lift.liftLatchOpened);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Acc Position:", robot.acc.accDrive.getCurrentPosition());
            telemetry.addData("Lift Position:", robot.lift.liftDrive.getCurrentPosition());
            telemetry.update();
        }
    }

}
