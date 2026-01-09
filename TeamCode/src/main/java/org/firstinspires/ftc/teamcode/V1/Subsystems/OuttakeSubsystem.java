package org.firstinspires.ftc.teamcode.V1.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

/*
This is the subsystem for the outtake of V1. It includes:
flywheel motor: can be set to a given speed in rpm
turret motor: can set power - determined by PID in AimTurret command
hood servo: can be set to a given position
kicker servo: can be kicked up and brought down
limelight: method to get data from limelight as LLResult

It also stores a variable targetSpeed, which is the current speed the flywheel
should be spinning at, and this can be compared to the actual flywheel speed
 */

public class OuttakeSubsystem extends SubsystemBase {
    public DcMotorEx flywheel;
    public DcMotor turret;
    public Servo hood;
    public Servo kicker;
    public Limelight3A limelight;

    private final int ticksPerRev = (int) flywheel.getMotorType().getTicksPerRev();
    private double targetSpeed;

    public OuttakeSubsystem(HardwareMap hwMap) {
        flywheel = hwMap.get(DcMotorEx.class, "flywheel");
        turret = hwMap.get(DcMotor.class, "turret");
        hood = hwMap.get(Servo.class, "hood");
        kicker = hwMap.get(Servo.class, "kicker");
        limelight = hwMap.get(Limelight3A.class, "limelight");

        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        limelight.pipelineSwitch(0);

        targetSpeed = 0;
    }

    // set flywheel to a speed in rpm
    public void setFlywheelSpeed(double speed) {
        double ticksPerSec = speed / 60 * ticksPerRev;
        flywheel.setVelocity(ticksPerSec);
        targetSpeed = speed;
    }

    public void setFlywheelPower(double power) {
        flywheel.setPower(power);
    }

    // get flywheel speed in rpm
    public double getFlywheelSpeed() {
        return flywheel.getVelocity() * 60 / ticksPerRev;
    }

    // get the speed the flywheel should be spinning at
    public double getTargetSpeed() {
        return targetSpeed;
    }

    // set the power of the turret motor
    public void powerTurret(double power) {
        turret.setPower(power);
    }

    public void rotateTurret(double angle) {
        turret.setTargetPosition((int) (angle / 360 * ticksPerRev));
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.5);
    }

    // set the position of the hood - to be used in the future
    public void setHood(double angle) {
        hood.setPosition(angle);
    }

    // rotate the kicker to kick an artifact to the outtake
    public void kickUp() {
        kicker.setPosition(0.8);
    }

    // move kicker back down to original position
    public void resetKicker() {
        kicker.setPosition(0.5);
    }

    // get the position of the kicker
    public double getKickerPos() {
        return kicker.getPosition();
    }

    // get limelight data as LLResult
    public LLResult readLimelight() {
        return limelight.getLatestResult();
    }

    public int getApriltagID() {
        LLResult llResult = readLimelight();
        int id = 0;
        if(llResult != null && llResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            if (!fiducials.isEmpty()) {
                id = fiducials.get(0).getFiducialId();
            }
        }
        return id;
    }

    // start the limelight - not done in constructor b/c limelight uses up energy
    public void startLimelight() {
        limelight.start();
    }
}
