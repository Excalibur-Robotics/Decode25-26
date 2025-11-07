package org.firstinspires.ftc.teamcode.DECODE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DrivetrainSubsystem extends SubsystemBase {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public DrivetrainSubsystem(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public void powerMotors(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void moveRobot(double x, double y, double yaw) {
        // deadzone
        if(Math.abs(x) < 0.05)
            x = 0;
        if(Math.abs(y) < 0.05)
            y = 0;
        if(Math.abs(yaw) < 0.05)
            yaw = 0;

        // calculate power of each motor
        double den = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(yaw), 1);
        double topLeftPower = (y + x + yaw) / den;
        double bottomLeftPower = (y - x + yaw) / den;
        double topRightPower = (y - x - yaw) / den;
        double bottomRightPower = (y + x - yaw) / den;

        // set power of each motor
        powerMotors(topLeftPower, topRightPower, bottomLeftPower, bottomRightPower);
    }
}
