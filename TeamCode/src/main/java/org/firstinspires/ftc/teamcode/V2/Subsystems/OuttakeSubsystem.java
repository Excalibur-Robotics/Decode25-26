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
    private int targetSpeed; // current speed the flywheel is trying to reach
    public static int presetFlywheelSpeed = 600; // preset flywheel speed

    public static double kickerDist = 0.7; // difference of up and down position
    public static double kickerDown = 0.05; // kicker servo down position
    public static double transferTime = 550; // in milliseconds

    private final int turretTicksPerRev = 2151;
    private LHV2PID turretPID;
    public static double kP = 0.014;
    public static double kI = 0.0;
    public static double kD = 0.001;

    private boolean onRedTeam;

    public OuttakeSubsystem(HardwareMap hwMap) {
        flywheel = hwMap.get(DcMotorEx.class, "flywheel");
        turret = hwMap.get(DcMotor.class, "turret");
        hoodR = hwMap.get(Servo.class, "hoodR");
        hoodL = hwMap.get(Servo.class, "hoodL");
        kicker = hwMap.get(Servo.class, "kicker");
        limelight = hwMap.get(Limelight3A.class, "limelight");

        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        limelight.pipelineSwitch(1);
        limelight.start();
        hoodL.setDirection(Servo.Direction.FORWARD);
        hoodR.setDirection(Servo.Direction.REVERSE);
        kicker.setDirection(Servo.Direction.REVERSE);

        targetSpeed = 0;
        turretPID = new LHV2PID(kP, kI, kD);
    }

    public void setFlywheelPower(double power) {
        flywheel.setPower(power);
    }

    public void setTargetSpeed(int speed) {
        targetSpeed = speed;
    }

    // calculate flywheel speed based on april tag
    // still have to figure this out
    public void calculateFlywheelSpeed() {
        setTargetSpeed(presetFlywheelSpeed);
    }

    // get flywheel speed in rpm
    public double getFlywheelSpeed() {
        return flywheel.getVelocity() * 60.0 / fwTicksPerRev;
    }

    // get the speed the flywheel should be spinning at
    public double getTargetSpeed() {
        return targetSpeed;
    }

    public int getFWTicksPerRev() {
        return fwTicksPerRev;
    }

    public void setKicker(double pos) {
        kicker.setPosition(pos);
    }

    // rotate the kicker to kick an artifact to the outtake
    public void kickUp() {
        kicker.setPosition(kickerDown + kickerDist);
    }

    // move kicker back down to original position
    public void resetKicker() {
        kicker.setPosition(kickerDown);
    }

    // get the position of the kicker
    public double getKickerPos() {
        return kicker.getPosition();
    }

    public double getKickerDist() {
        return kickerDist;
    }
    public double getKickerDown() {
        return kickerDown;
    }
    public double getTransferTime() {
        return transferTime;
    }

    // set the power of the turret motor
    public void powerTurret(double power) {
        if(!(getTurretPos() > 110 && power > 0) &&
                !(getTurretPos() < -90 && power < 0)) {
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

    public double calculateHood() {
        return 0.45;
    }

    public double getHoodAngle() {
        return hoodR.getPosition();
    }

    public void setTeam(boolean redTeam) {
        onRedTeam = redTeam;
        limelight.pipelineSwitch(onRedTeam ? 1 : 2);
    }

    public double getTX() {
        double tx = 0;
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            tx = llResult.getTx();
        }
        return tx;
    }
    public double getTA() {
        double ta = 0;
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            ta = llResult.getTa();
        }
        return ta;
    }

    public int getApriltagID() {
        LLResult llResult = limelight.getLatestResult();
        int id = 0;
        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            if (!fiducials.isEmpty()) {
                id = fiducials.get(0).getFiducialId();
            }
        }
        return id;
    }

    @Override
    public void periodic() {
        calculateFlywheelSpeed();
    }

    // start the limelight - not done in constructor b/c limelight uses up energy
    public void startLimelight() {
        limelight.start();
    }
}
