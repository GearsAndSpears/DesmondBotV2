package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Accumulator extends BaseHardware{

    private double frontAccIntake = 1;
    private double backAccIntake = 1;

    private double frontAccOutput = -1;
    private double backAccOutput = -1;

    private final int deployedPosition = 400;
    private final int retractedPosition = 0;
    private final int collectingPosition = 0;

    Telemetry telemetry;

    public Accumulator(){

    }

    public DcMotorEx accDrive;
    public CRServo frontAcc, backAcc;

    public enum accDrivePosition{
        RETRACTED,
        DEPLOYED,
        COLLECTING
    }

    accDrivePosition acc = accDrivePosition.RETRACTED;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){
        accDrive = (DcMotorEx)hardwareMap.dcMotor.get("accDrive");

        frontAcc = hardwareMap.crservo.get("frontAcc");
        backAcc = hardwareMap.crservo.get("backAcc");

        accDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        accDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        accDrive.setDirection(DcMotorEx.Direction.REVERSE);

        accDrive.setPower(0);

        frontAcc.setDirection(CRServo.Direction.FORWARD);
        backAcc.setDirection(CRServo.Direction.FORWARD);
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
        accDrive.setPower(0.3);
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

    }

}
