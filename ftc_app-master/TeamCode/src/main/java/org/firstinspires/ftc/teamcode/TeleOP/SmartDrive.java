package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

public class SmartDrive extends LinearOpMode {

    //Variable Declaration
    Robot robot = new Robot();
    boolean xCurrentlyPressed = false;
    boolean xState = false;

    @Override
    public void runOpMode() {
        //on init code
        robot.init(hardwareMap, telemetry);
        init();

        waitForStart();

        while (opModeIsActive()){
            //main loop code
            loop();
            robot.driveTrain.manualDrive(gamepad1);
            if(gamepad2.x && !xCurrentlyPressed){
                xCurrentlyPressed = true;
                if(xState){
                    xState = false;
                }
                else {
                    xState = true;
                }
            }
            if(!gamepad2.x){
                xCurrentlyPressed = false;
            }

            if(xState){

            }
            else {
                robot.lift.manualControl(gamepad2);
                robot.acc.manualControl(gamepad2);
            }
        }

    }
}
