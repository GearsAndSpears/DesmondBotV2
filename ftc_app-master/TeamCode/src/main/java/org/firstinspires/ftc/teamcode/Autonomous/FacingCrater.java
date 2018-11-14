package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.Hardware.Accumulator.accDrivePosition;
import static org.firstinspires.ftc.teamcode.Hardware.DriveTrain.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.Hardware.DriveTrain.TURN_SPEED;

@Autonomous(name="FacingCrater V4", group="Pushbot")

public class FacingCrater extends LinearOpMode {

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
        robot.auto.gyroTurn(TURN_SPEED, 0);
        robot.auto.gyroHold(TURN_SPEED, 0, .5);
        robot.auto.gyroDrive(DRIVE_SPEED, 9, 0);

        //turn left and drive towards wall
        robot.auto.gyroTurn(TURN_SPEED, 90);
        robot.auto.gyroHold(TURN_SPEED, 90, .5);
        robot.auto.gyroDrive(DRIVE_SPEED, 47, 90);

        //face depot and claim
        robot.auto.gyroTurn(TURN_SPEED, 135);
        robot.auto.gyroHold(TURN_SPEED, 135, .5);
        robot.auto.gyroDrive(DRIVE_SPEED, 25, 135);
        robot.acc.output();

        sleep(1000);

        //back into crater
        robot.auto.gyroDrive(DRIVE_SPEED, -60, 135);


    }
}
