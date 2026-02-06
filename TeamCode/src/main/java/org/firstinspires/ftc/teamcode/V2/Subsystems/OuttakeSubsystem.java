package org.firstinspires.ftc.teamcode.V2.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.V2.LHV2PID;

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

@Config
public class OuttakeSubsystem extends SubsystemBase {
    public DcMotorEx flywheel;
    public DcMotor turret;
    public Servo hoodR;
    public Servo hoodL;
    public Servo kicker;
    public Limelight3A limelight;

    private final int fwTicksPerRev = 112;
    private double targetSpeed;

    public static double kickerUp = 0.7;
    public static double kickerDown = 0;

    private final int turretTicksPerRev = 112; // need to look up
    private LHV2PID turretPID;
    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.0001;

    public OuttakeSubsystem(HardwareMap hwMap) {
        flywheel = hwMap.get(DcMotorEx.class, "flywheel");
        turret = hwMap.get(DcMotor.class, "turret");
        hoodR = hwMap.get(Servo.class, "hoodR");
        hoodL = hwMap.get(Servo.class, "hoodL");
        kicker = hwMap.get(Servo.class, "kicker");
        limelight = hwMap.get(Limelight3A.class, "limelight");

        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        limelight.pipelineSwitch(0);
        limelight.start();
        hoodL.setDirection(Servo.Direction.FORWARD);
        hoodR.setDirection(Servo.Direction.REVERSE);

        targetSpeed = 0;
        turretPID = new LHV2PID(kP, kI, kD);
    }

    // set flywheel to a speed in rpm
    public void setFlywheelSpeed(double speed) {
        double ticksPerSec = speed / 60 * fwTicksPerRev;
        flywheel.setVelocity(ticksPerSec);
        targetSpeed = speed;
    }

    public void setFlywheelPower(double power) {
        flywheel.setPower(power);
    }

    // get flywheel speed in rpm
    public double getFlywheelSpeed() {
        return flywheel.getVelocity() * 60.0 / fwTicksPerRev;
    }

    // get the speed the flywheel should be spinning at
    public double getTargetSpeed() {
        return targetSpeed;
    }

    public int getFwTicksPerRev() {
        return fwTicksPerRev;
    }

    public void setKicker(double pos) {
        kicker.setPosition(pos);
    }

    // rotate the kicker to kick an artifact to the outtake
    public void kickUp() {
        kicker.setPosition(kickerUp);
    }

    // move kicker back down to original position
    public void resetKicker() {
        kicker.setPosition(kickerDown);
    }

    // get the position of the kicker
    public double getKickerPos() {
        return kicker.getPosition();
    }

    public double getKickerUp() {
        return kickerUp;
    }
    public double getKickerDown() {
        return kickerDown;
    }

    // set the power of the turret motor
    public void powerTurret(double power) {
        if(!(turret.getCurrentPosition() > 110 && power < 0) &&
                !(turret.getCurrentPosition() < -110 && power > 0)) {
            turret.setPower(power);
        }
    }

    // turret angle in degrees, straight forward is 0
    public double getTurretPos() {
        return turret.getCurrentPosition() * 360.0 / turretTicksPerRev;
    }

    // for apriltag: CP = tx
    // for specific angle: CP = angle - turret position
    public void calculateTurret(double CP) {
        double power = turretPID.Calculate(0, CP);
        powerTurret(power);
    }

    // set the position of the hood
    public void setHood(double angle) {
        hoodR.setPosition(angle);
        hoodL.setPosition(angle);
    }

    public double getHoodAngle() {
        return hoodR.getPosition();
    }

    public double getTX() {
        double tx = 0;
        LLResult llData = limelight.getLatestResult();
        if (llData != null && llData.isValid()) {
            tx = llData.getTx();
        }
        return tx;
    }

    public int getApriltagID() {
        LLResult llResult = limelight.getLatestResult();
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
