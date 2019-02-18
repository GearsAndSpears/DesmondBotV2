package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.abs;

public class Accumulator extends BaseHardware{

    private double frontAccIntake = 1;
    private double backAccIntake = 1;

    private double frontAccOutput = -1;
    private double backAccOutput = -1;

    public final int deployedPosition =1300;
    public int retractedPosition = 540;
    public final int collectingPosition = 1470;
    public final int storingPosition = 460;

    public final int slidePartiallyRetracted = 100;
    public final int slideTotallyRetracted = 0;
    public final int slideTotallyExtended = 200;
    public final double slidePower = .3;

    private boolean upPressed = false;
    private boolean downPressed = false;

    Telemetry telemetry;

    public DcMotorEx accDrive, accSlide;
    public CRServo frontAcc, backAcc;

    public enum accDrivePosition{
        STORING,
        RETRACTED,
        DEPLOYED,
        COLLECTING
    }

    accDrivePosition acc = accDrivePosition.RETRACTED;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){
        accDrive = (DcMotorEx)hardwareMap.dcMotor.get("acc_drive");
        accSlide = (DcMotorEx)hardwareMap.dcMotor.get("acc_slide");

        frontAcc = hardwareMap.crservo.get("front_acc");
        backAcc = hardwareMap.crservo.get("back_acc");

        accDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        accSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        accDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        accSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        accDrive.setDirection(DcMotorEx.Direction.REVERSE);
        accSlide.setDirection(DcMotorEx.Direction.FORWARD);

        accDrive.setPower(0);
        accSlide.setPower(0);

        frontAcc.setDirection(CRServo.Direction.REVERSE);
        backAcc.setDirection(CRServo.Direction.FORWARD);

        accDrive.setTargetPositionTolerance(1);
        accSlide.setTargetPositionTolerance(1);

    }

    public void setup(){
        accDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        accSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        accDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        accSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        accDrive.setPower(0.3);

        setArmState(accDrivePosition.RETRACTED);
    }

    public void intake(){
        frontAcc.setPower(1);
        backAcc.setPower(1);
    }

    public void output(){
        frontAcc.setPower(-1);
        backAcc.setPower(-1);
    }

    public void stop(){
        frontAcc.setPower(0);
        backAcc.setPower(0);
    }

    public void setArmState(accDrivePosition adp){
        accDrive.setPower(0.4);
        switch (adp){
            case RETRACTED:
                accDrive.setTargetPosition(retractedPosition);
                this.stop();
                break;
            case DEPLOYED:
                accDrive.setTargetPosition(deployedPosition);
                this.intake();
                break;
            case COLLECTING:
                accDrive.setTargetPosition(collectingPosition);
                this.intake();
                break;
        }
    }

    public void manualControl(Gamepad gamepad){

        if(gamepad.dpad_up && !upPressed){
            if(acc == accDrivePosition.COLLECTING){
                acc = accDrivePosition.DEPLOYED;
            }
            else{
                acc = accDrivePosition.RETRACTED;
            }
            upPressed = true;
        }
        else if(gamepad.dpad_down && !downPressed){
            if(acc == accDrivePosition.RETRACTED){
                acc = accDrivePosition.DEPLOYED;
            }
            
            else if(acc == accDrivePosition.DEPLOYED){
                acc = accDrivePosition.COLLECTING;
            }
            downPressed = true;
        }

        if(!gamepad.dpad_up){
            upPressed = false;
        }

        if(!gamepad.dpad_down){
            downPressed = false;
        }

        this.setArmState(acc);

        if(gamepad.left_stick_y > 0){
            this.accSlide.setTargetPosition(slideTotallyExtended);
        }
        else if(gamepad.left_stick_y < 0){
            this.accSlide.setTargetPosition(slidePartiallyRetracted);
        }
        this.accSlide.setPower(abs(gamepad.left_stick_y));
    }

}
