package org.firstinspires.ftc.teamcode.DECODE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class In_take extends SubsystemBase {
    DcMotor intake1;
    DcMotor intake2;

    public In_take(HardwareMap hardware){
        intake1 = hardware.get(DcMotor.class,"intake1");
        intake2 = hardware.get(DcMotor.class,"intake2");

        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.REVERSE);
    }

    public void movement (double power){
        intake1.setPower(power);
        intake2.setPower(power);
    }
}
