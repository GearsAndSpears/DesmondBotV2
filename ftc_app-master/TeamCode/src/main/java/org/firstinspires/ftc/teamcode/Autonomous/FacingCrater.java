package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.Hardware.Accumulator.accDrivePosition;
import static org.firstinspires.ftc.teamcode.Hardware.DriveTrain.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.Hardware.DriveTrain.TURN_SPEED;

@Autonomous(name="FacingCrater V3", group="Pushbot")

public class FacingCrater extends BaseAuto {

    public Robot robot = new Robot();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        robot.acc.setArmState(accDrivePosition.RETRACTED);

        robot.vision.detector.enable();

        telemetry.addData("Robot Ready", "");
        telemetry.update();

        waitForStart();

        sample();

        //set up for first motion
        gyroTurn(TURN_SPEED, 0);
        gyroHold(TURN_SPEED, 0, .5);
        gyroDrive(DRIVE_SPEED, 11, 0);

        //turn left and drive towards wall
        gyroTurn(TURN_SPEED, 90);
        gyroHold(TURN_SPEED, 90, .5);
        gyroDrive(DRIVE_SPEED, 47, 90);

        //face depot and claim
        gyroTurn(TURN_SPEED, 135);
        gyroHold(TURN_SPEED, 135, .5);
        gyroDrive(DRIVE_SPEED, 25, 135);
        robot.acc.output();

        //back into crater
        gyroDrive(DRIVE_SPEED, -35, 135);


    }
}
