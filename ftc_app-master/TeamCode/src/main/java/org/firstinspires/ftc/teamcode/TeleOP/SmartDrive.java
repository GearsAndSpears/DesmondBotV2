package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Accumulator;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

public class SmartDrive extends LinearOpMode {

    //Variable Declaration
    Robot robot = new Robot();
    boolean xCurrentlyPressed = false;
    boolean xState = false;

    private State currentState;

    private enum State{
        STATE_INITIAL,
        STATE_RETRACTING,
        STATE_COLLECTING,
        STATE_STORING,
        STATE_LIFTING,
        STATE_DUMPING,
        STATE_RETURNING
    }

    private void newState(State state){
        currentState = state;
    }

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

            if(!gamepad2.x){
                xCurrentlyPressed = false;
            }

            switch (currentState){
                case STATE_INITIAL:
                    robot.acc.manualControl(gamepad2);
                    robot.lift.manualControl(gamepad2);
                    if(gamepad2.x && !xCurrentlyPressed){
                        newState(State.STATE_RETRACTING);
                    }
                    break;

                case STATE_RETRACTING:
                    robot.acc.accSlide.setPower(robot.acc.slidePower);
                    robot.acc.accSlide.setTargetPosition(robot.acc.slidePartiallyRetracted);

                    robot.lift.liftDrive.setTargetPosition(robot.lift.liftRetracted);
                    robot.lift.dumpBucket.setPosition(robot.lift.dumpBucketRetracted);

                    if(robot.acc.accSlide.getCurrentPosition() == robot.acc.slidePartiallyRetracted){
                        newState(State.STATE_COLLECTING);
                    }
                    if(gamepad2.x && !xCurrentlyPressed){
                        newState(State.STATE_INITIAL);
                    }
                    break;

                case STATE_STORING:
                    robot.acc.setArmState(Accumulator.accDrivePosition.STORING);
                    if(robot.acc.accDrive.getCurrentPosition() == robot.acc.storingPosition){
                        robot.acc.setArmState(Accumulator.accDrivePosition.RETRACTED);
                        robot.lift.dumpBucket.setPosition(robot.lift.dumpBucketCentered);
                    }
                    if(robot.lift.dumpBucket.getPosition() == robot.lift.dumpBucketCentered){
                        newState(State.STATE_LIFTING);
                    }
                    if(gamepad2.x && !xCurrentlyPressed){
                        newState(State.STATE_INITIAL);
                    }
                    break;

                case STATE_LIFTING:
                    robot.lift.liftDrive.setTargetPosition(robot.lift.liftExtended);
                    if(robot.lift.liftDrive.getCurrentPosition() == robot.lift.liftExtended){
                        newState(State.STATE_DUMPING);
                    }
                    if(gamepad2.x && !xCurrentlyPressed){
                        newState(State.STATE_INITIAL);
                    }
                    break;

                case STATE_DUMPING:


            }
        }

    }
}
