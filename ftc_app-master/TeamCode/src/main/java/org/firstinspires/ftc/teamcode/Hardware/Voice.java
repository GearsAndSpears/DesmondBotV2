package org.firstinspires.ftc.teamcode.Hardware;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;

public class Voice extends BaseHardware {

    SoundPlayer voice;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    File takingThis = new File("org/firstinspires/ftc/teamcode/Sounds/taking-this.wav");

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap ahardwareMap, Telemetry atelemetry){
        hardwareMap = ahardwareMap;
        telemetry = atelemetry;
    }

    public void manualControl(Gamepad gamepad){
        if(gamepad.dpad_up){
            voice.startPlaying(hardwareMap.appContext, takingThis);
        }
    }
}
