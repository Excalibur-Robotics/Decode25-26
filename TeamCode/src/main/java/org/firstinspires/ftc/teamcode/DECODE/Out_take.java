package org.firstinspires.ftc.teamcode.DECODE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Out_take extends SubsystemBase {
    DcMotor out_take;
    DcMotor out_take2;

    public Out_take(HardwareMap hardware){
        out_take = hardware.get(DcMotor.class, "outtakeR");
        out_take2 = hardware.get(DcMotor.class,"outtakeL");

        out_take.setDirection(DcMotor.Direction.FORWARD);
        out_take2.setDirection(DcMotor.Direction.FORWARD);
    }
    public void basicMovement (double power){

        out_take.setPower(-power);
        out_take2.setPower(power);
    }
}
