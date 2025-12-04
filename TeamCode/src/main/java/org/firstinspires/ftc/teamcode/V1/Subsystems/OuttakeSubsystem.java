package org.firstinspires.ftc.teamcode.V1.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeSubsystem extends SubsystemBase {
    public DcMotor flywheel;
    public DcMotor turret;
    public Servo hood;
    public Servo kicker;
    public Limelight3A limelight;

    public OuttakeSubsystem(HardwareMap hwMap) {
        flywheel = hwMap.get(DcMotor.class, "flywheel");
        turret = hwMap.get(DcMotor.class, "turret");
        hood = hwMap.get(Servo.class, "hood");
        kicker = hwMap.get(Servo.class, "kicker");
        limelight = hwMap.get(Limelight3A.class, "limelight");

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setFlywheelSpeed(double speed) {
        flywheel.setPower(speed);
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

    public LLResult readLimelight() {
        return limelight.getLatestResult();
    }
}
