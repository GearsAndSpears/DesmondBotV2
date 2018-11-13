package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

import static org.firstinspires.ftc.teamcode.Hardware.Accumulator.accDrivePosition;
import static org.firstinspires.ftc.teamcode.Hardware.DriveTrain.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.Hardware.DriveTrain.TURN_SPEED;

@Autonomous(name="FacingDepot V4", group="Pushbot")

public class FacingDepot extends LinearOpMode {

    Robot robot = new Robot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        robot.setup(hardwareMap, telemetry, this);

        robot.vision.detector.enable();

        telemetry.addData("Robot Ready", "");
        telemetry.update();

        waitForStart();

        robot.acc.setArmState(accDrivePosition.RETRACTED);

        //land();
        robot.auto.land();
        //sample();
        robot.auto.sample();
        //set up for first motion
        //set up first motion

        robot.auto.gyroTurn(TURN_SPEED, 0);
        robot.auto.gyroHold(TURN_SPEED, 0, .5);
        robot.auto.gyroDrive(DRIVE_SPEED, 11, 0);

        //turn left and drive towards wall
        robot.auto.gyroTurn(TURN_SPEED, 75);
        robot.auto.gyroHold(TURN_SPEED, 75, .5);
        robot.auto.gyroDrive(DRIVE_SPEED, 36, 75);

        //turn towards depot and claim
        robot.auto.gyroTurn(TURN_SPEED, -45);
        robot.auto.gyroHold(TURN_SPEED, -45, .5);
        robot.auto.gyroDrive(DRIVE_SPEED, 35, -45);
        robot.acc.output();
        //back into crater
        robot.auto.gyroDrive(DRIVE_SPEED, -60, -45);

    }
}
