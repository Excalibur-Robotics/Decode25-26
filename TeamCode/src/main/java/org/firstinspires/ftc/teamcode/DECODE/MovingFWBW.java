package org.firstinspires.ftc.teamcode.DECODE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MovingFWBW extends SubsystemBase {
    DcMotor topRight;
    DcMotor topLeft;
    DcMotor bottomRight;
    DcMotor bottomLeft;

    public MovingFWBW(HardwareMap hardware){
        topRight = hardware.get(DcMotor.class, "frontRight");
        topLeft = hardware.get(DcMotor.class,"frontLeft");
        bottomLeft = hardware.get(DcMotor.class,"backLeft");
        bottomRight = hardware.get(DcMotor.class,"backRight");

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

/*
The code in this subsystem
sets the motor direction and power
to allow it to move
forward and backward

Line 8-12 - Defines the motors
Lines 14-23 - Motors names are set and direction of motors are set
Lines 26-30 - Sets motor powers
 */