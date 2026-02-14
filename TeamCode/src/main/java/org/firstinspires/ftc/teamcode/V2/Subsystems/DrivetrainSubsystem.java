package org.firstinspires.ftc.teamcode.V2.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
This the subsystem for a mecanum drivetrain. It includes the 4 drive motors,
and in the future it will include Pinpoint for odometry. It has a method to
move the robot based on given x, y, and yaw values. Its periodic method takes
input from the gamepad to move the robot, and this is automatically called each
loop in teleop.
 */

public class DrivetrainSubsystem extends SubsystemBase {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public DrivetrainSubsystem(HardwareMap hwMap) {
        // initialize motors
        frontLeft = hwMap.get(DcMotor.class, "flwheel");
        frontRight = hwMap.get(DcMotor.class, "frwheel");
        backLeft = hwMap.get(DcMotor.class, "blwheel");
        backRight = hwMap.get(DcMotor.class, "brwheel");

        // reverse left motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    // power motors
    public void powerMotors(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    // calculate powers of motors for mecanum drive
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

    // periodic method runs each time scheduler is run in the opmode
    // takes gamepad joysticks to feed to moveRobot()
    public void teleOpDrive(Gamepad gamepad) {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double yaw = gamepad.right_stick_x;

        moveRobot(x, y, yaw);
    }
    // field centric driving - pass the heading
    public void teleOpDrive(Gamepad gamepad, double heading) {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double yaw = gamepad.right_stick_x;

        double newX = x*Math.cos(heading) + y*Math.sin(heading);
        double newY = y*Math.cos(heading) - x*Math.sin(heading);

        moveRobot(newX, newY, yaw);
    }
}
