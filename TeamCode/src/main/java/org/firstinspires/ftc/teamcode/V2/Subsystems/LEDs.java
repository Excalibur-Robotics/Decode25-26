package org.firstinspires.ftc.teamcode.V2.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class LEDs extends SubsystemBase {
    LED green;
    LED red;
    LED purple;

    public LEDs (HardwareMap hardware){
        green = hardware.get(LED.class,"GreenLED");
        //red = hardware.get(LED.class,"RedLED");
        //purple = hardware.get(LED.class,"purpleLED");
    }

    public void green() {
        green.on();
        //red.off();
        //purple.off();
    }

    public void red(){
        green.off();
        //red.on();
        //purple.off();
    }

    public void purple(){
        green.off();
        //red.off();
        //purple.on();
    }

    public void off() {
        green.off();
        red.off();
        purple.off();
    }

}


