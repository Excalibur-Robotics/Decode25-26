package org.firstinspires.ftc.teamcode.DECODE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MovingFWBW extends SubsystemBase {
    DcMotor topRight;
    DcMotor topLeft;
    DcMotor bottomRight;
    DcMotor bottomLeft;

    public MovingFWBW(HardwareMap hardware){
        topRight = hardware.get(DcMotor.class, "topRight");
        topLeft = hardware.get(DcMotor.class,"topLeft");
        bottomLeft = hardware.get(DcMotor.class,"bottomLeft");
        bottomRight = hardware.get(DcMotor.class,"bottomRight");

        topRight.setDirection(DcMotor.Direction.FORWARD);
        topLeft.setDirection(DcMotor.Direction.FORWARD);
        bottomLeft.setDirection(DcMotor.Direction.FORWARD);
        bottomRight.setDirection(DcMotor.Direction.FORWARD);
    }

    public void movement (double power){
        topRight.setPower(power);
        topLeft.setPower(-power);
        bottomRight.setPower(power);
        bottomLeft.setPower(-power);
    }
}