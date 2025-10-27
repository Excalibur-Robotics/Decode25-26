package org.firstinspires.ftc.teamcode.EvanTeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    public DcMotor topLeftDriveMotor;
    public DcMotor bottomLeftDriveMotor;
    public DcMotor topRightDriveMotor;
    public DcMotor bottomRightDriveMotor;

    private IMUClass imu = new IMUClass();

    HardwareMap hwMap;

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        // initialize motors
        topLeftDriveMotor = hwMap.get(DcMotor.class, "frontLeft");
        bottomLeftDriveMotor = hwMap.get(DcMotor.class, "backLeft");
        topRightDriveMotor = hwMap.get(DcMotor.class, "frontRight");
        bottomRightDriveMotor = hwMap.get(DcMotor.class, "backRight");

        // set motors to not use encoders
        topLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // reverse left motors
        topLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        topRightDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        bottomRightDriveMotor.setDirection(DcMotor.Direction.FORWARD);

        // set motors to brake when power is set to zero
        topLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set power to zero
        topLeftDriveMotor.setPower(0);
        bottomLeftDriveMotor.setPower(0);
        topRightDriveMotor.setPower(0);
        bottomRightDriveMotor.setPower(0);

        imu.init(hwMap);
    }

    // Mecanum drive
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
        topLeftDriveMotor.setPower(topLeftPower);
        topRightDriveMotor.setPower(topRightPower);
        bottomLeftDriveMotor.setPower(bottomLeftPower);
        bottomRightDriveMotor.setPower(bottomRightPower);
    }

    // adjust coordinates for field relative driving
    public void fieldRelativeDrive(double x, double y, double yaw) {
        double heading = imu.getHeading();
        double newX = x*Math.cos(heading) + y*Math.sin(heading);
        double newY = y*Math.cos(heading) - x*Math.sin(heading);

        moveRobot(newX, newY, yaw);
    }

    // get the heading of the robot
    public double getHeading() {
        return imu.getHeading();
    }
}