package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    public DriveTrain driveTrain = new DriveTrain();
    public Gyro gyro = new Gyro();
    public Accumulator acc = new Accumulator();

    public Robot(){

    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){
        driveTrain.init(hardwareMap, telemetry);
        gyro.init(hardwareMap, telemetry);
        acc.init(hardwareMap, telemetry);
    }

}
