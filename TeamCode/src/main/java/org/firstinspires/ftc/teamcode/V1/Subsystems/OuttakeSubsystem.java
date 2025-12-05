package org.firstinspires.ftc.teamcode.V1.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        limelight.pipelineSwitch(0);

        targetSpeed = 0;
    }

    // speed in rpm
    public void setFlywheelSpeed(double speed) {
        double ticksPerSec = speed / 60 * ticksPerRev;
        flywheel.setVelocity(ticksPerSec);
        targetSpeed = speed;
    }

    public double getFlywheelSpeed() {
        return flywheel.getVelocity() * 60 / ticksPerRev;
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void powerTurret(double power) {
        turret.setPower(power);
    }

    public void setHood(double angle) {
        hood.setPosition(angle);
    }

    public void kickUp() {
        kicker.setPosition(0.5);
    }

    public void kickerDown() {
        kicker.setPosition(0);
    }

    public double getKickerPos() {
        return kicker.getPosition();
    }

    public LLResult readLimelight() {
        return limelight.getLatestResult();
    }

    public void startLimelight() {
        limelight.start();
    }
}
