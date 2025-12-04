package org.firstinspires.ftc.teamcode.Spindexer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;


// This is the Spindex code so far :)
// Feel free to make changes and tell me improvements

@TeleOp(name="Yeet")
public class Spindex extends OpMode {
    /* Color Sensor Variables*/
    private NormalizedColorSensor color_sensor1;

    private NormalizedColorSensor color_sensor2;
    double hue_slot1; // Will be used to store hue value for checking slots
    double hue_slot2; // Same as hue_slot1

    @Override
    public void init() {
        /*Mapping Variables*/
        color_sensor1=hardwareMap.get(NormalizedColorSensor.class, "CS1");
        color_sensor2=hardwareMap.get(NormalizedColorSensor.class, "CS2");


    }

    public void loop() {
        /*NormalizedRGBA  */
        NormalizedRGBA color_slot1= color_sensor1.getNormalizedColors();
        NormalizedRGBA color_slot2= color_sensor2.getNormalizedColors();
        /* JavaUtil.colorToHue() converts RGBA value into a hue number from 0 to 360
           Used later for checking the color of the balls*/
        hue_slot1= JavaUtil.colorToHue(color_slot1.toColor());
        hue_slot2= JavaUtil.colorToHue(color_slot2.toColor());
        telemetry.addData("hue1: ", hue_slot1); // Shows hue number
        telemetry.addData("hue2: ", hue_slot2);

        //Slot 1 (Intake Slot) Checks slot using hue value

        if (hue_slot1<=245 && hue_slot1>=210){
            telemetry.addData("Slot 1: ", "Purple");


        } else if (hue_slot1<=180 && hue_slot1>=150) {
            telemetry.addData("Slot 1: ", "Green");

        } else {
            telemetry.addData("Slot 1: ", "Unknown");
        }

        //Slot 2 (Non-shooter slot)

        if (hue_slot2<=245 && hue_slot2>=210) {
            telemetry.addData("Slot 2: ", "Purple");
        } else if (hue_slot2<=180 && hue_slot2>=150) {
            telemetry.addData("Slot 2: ", "Green");

        } else {
            telemetry.addData("Slot 2: ", "Unknown");
        }
        telemetry.update();
        /* A big problem in this color sensor process occurs when
           the hole of the ball happens to on the color sensor,
           making the color display as "Unknown." Hopefully,
           the second color sensor will be able to check the
           value if the first one fails because of this.

           Additionally, I don't know quite yet how to "store"
           the value from one slot. This would be important later
           when the user spins the spindex.
         */
        
    }

}
