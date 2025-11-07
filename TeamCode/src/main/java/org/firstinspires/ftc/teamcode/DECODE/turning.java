package org.firstinspires.ftc.teamcode.DECODE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class turning extends SubsystemBase {
    DcMotor topRight;
    DcMotor topLeft;
    DcMotor bottomRight;
    DcMotor bottomLeft;

    public turning (HardwareMap hardware){
        topRight = hardware.get(DcMotor.class,"frontRight");
        topLeft = hardware.get(DcMotor.class,"frontLeft");
        bottomRight = hardware.get(DcMotor.class,"backRight");
        bottomLeft = hardware.get(DcMotor.class,"backLeft");

        topLeft.setDirection(DcMotor.Direction.REVERSE);
        topRight.setDirection(DcMotor.Direction.REVERSE);
        bottomRight.setDirection(DcMotor.Direction.FORWARD);
        bottomLeft.setDirection(DcMotor.Direction.FORWARD);
    }

    public void rturn (double power){
        topRight.setPower(-power);
        bottomRight.setPower(power);
        bottomLeft.setPower(power);
        bottomRight.setPower(-power);
    }

    public void lturn (double power){
        topLeft.setPower(power);
        bottomLeft.setPower(-power);
        bottomLeft.setPower(-power);
        bottomRight.setPower(power);
    }
}
