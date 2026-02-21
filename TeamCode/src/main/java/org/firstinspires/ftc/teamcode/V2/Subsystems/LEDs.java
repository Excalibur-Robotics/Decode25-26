package org.firstinspires.ftc.teamcode.V2.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

public class LEDs extends SubsystemBase {
    /*
    LED green;
    LED red;
    LED purple;
*/
    Servo led_light;

    public LEDs (HardwareMap hardware){
        led_light = hardware.get(Servo.class,"GreenLED");
    }

    public void green() {
        led_light.setPosition(0.5);
    }
/*
    public void red(){

    }

    public void purple(){

    }

    public void off() {

    }
*/
}


