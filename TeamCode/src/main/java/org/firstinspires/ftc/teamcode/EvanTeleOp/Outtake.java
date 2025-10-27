package org.firstinspires.ftc.teamcode.EvanTeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {
    public DcMotor midtake;
    public DcMotor shootR;
    public DcMotor shootL;
    private ElapsedTime shootTime = new ElapsedTime();
    private boolean shooting;

    public void init(HardwareMap hwMap) {
        shootR = hwMap.get(DcMotor.class, "outtakeR");
        shootL = hwMap.get(DcMotor.class, "outtakeL");
        midtake = hwMap.get(DcMotor.class, "midtake");

        shootR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        midtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooting = false;
    }

    public void shoot() {
        // turn on motors and start shoot timer
        if(!shooting) {
            shootR.setPower(-1);
            shootL.setPower(1);
            midtake.setPower(-1); // remove if using delay
            shootTime.reset();
            shooting = true;
        }
        // stop motors after 1.5 seconds
        else if(shootTime.seconds() > 1.5) {
            shootR.setPower(0);
            shootL.setPower(0);
            midtake.setPower(0);
            shooting = false;
        }
        /* add delay for motors to speed up (wait 0.5 s to power midtake)
        else if(shootTime.seconds() > 0.5) {
            midtake.setPower(-1);
        }
        */
    }

    // return whether robot is currently shooting
    public boolean getState() {
        return shooting;
    }
}
