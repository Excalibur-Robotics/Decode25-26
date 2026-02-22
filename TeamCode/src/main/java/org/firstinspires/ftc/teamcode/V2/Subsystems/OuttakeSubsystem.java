package org.firstinspires.ftc.teamcode.V2.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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
    public static int flywheelSpeedFar = 725;
    public static int flywheelSpeedClose = 575;

    public static double kickerDist = 1; // difference of up and down position
    public static double kickerDown = 0.0; // kicker servo down position
    public static double transferTime = 550; // in milliseconds

    private final int turretTicksPerRev = 2151;
    private LHV2PID turretPID;
    public static double kP = 0.018; // needs to be tuned
    public static double kI = 0.0;
    public static double kD = 0.5; // needs to be tuned

    public static double hoodPosFar = 0.45;
    public static double hoodPosClose = 0.15;

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
        hoodL.setDirection(Servo.Direction.FORWARD);
        hoodR.setDirection(Servo.Direction.REVERSE);
        kicker.setDirection(Servo.Direction.FORWARD);

        targetSpeed = flywheelSpeedClose;
        turretPID = new LHV2PID(kP, kI, kD);
        kicker.setPosition(kickerDown);
        setHood(hoodPosClose);
    }

    @Override
    public void periodic() {
        //calculateTurret(getTX());
        //calculateFlywheelSpeed();
        //setHood(calculateHood());
    }

    public void setFlywheelPower(double power) {
        flywheel.setPower(power);
    }

    public void setTargetSpeed(int speed) {
        targetSpeed = speed;
    }

    // calculate flywheel speed based on april tag
    public void calculateLaunch() {
        if(getTA() > 1) {
            setTargetSpeed(flywheelSpeedClose);
            setHood(hoodPosClose);
        }
        else if(getTA() < 0.8 && getTA() > 0) {
            setTargetSpeed(flywheelSpeedFar);
            setHood(hoodPosFar);
        }
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
        turret.setPower(power);
    }

    // aim turret with apriltag: CP = tx
    public void calculateTurretLL(double CP) {
        if(getTA() < 0.8 && getTA() > 0) { // offset angle for far shooting
            if(onRedTeam)
                turret.setPower(turretPID.Calculate(4, CP));
            else
                turret.setPower(turretPID.Calculate(-4, CP));
        }
        else if(getTA() > 1) {
            turret.setPower(turretPID.Calculate(0, CP));
        }
        else {
            turret.setPower(0);
        }
    }

    // input target angle
    public void rotateTurret(double angle) {
        if(angle > 100) {
            angle = 100;
        }
        else if(angle < -100) {
            angle = -100;
        }
        turret.setPower(turretPID.Calculate(angle, (double) turret.getCurrentPosition() / turretTicksPerRev * 360));
    }

    // aim turret with robot position
    public void aimTurret(Pose botPose) {
        Pose goal = new Pose(onRedTeam ? 136 : 8, 142);
        double angle = Math.atan((goal.getY() - botPose.getY()) / (goal.getX() - botPose.getX()));
        if(angle < 0)
            angle += 180;
        double turretAngle = angle - botPose.getHeading();
        rotateTurret(turretAngle);
    }

    public void resetTurretEncoder() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // turret angle in degrees, straight forward is 0
    public double getTurretPos() {
        return turret.getCurrentPosition() * 360.0 / turretTicksPerRev;
    }


    // set the position of the hood
    public void setHood(double angle) {
        hoodR.setPosition(angle);
        hoodL.setPosition(angle);
    }

    public double getHoodAngle() {
        return hoodR.getPosition();
    }
    public double getHoodFar() {
        return hoodPosFar;
    }
    public double getHoodClose() {
        return hoodPosClose;
    }

    public void setTeam(boolean redTeam) {
        onRedTeam = redTeam;
        limelight.pipelineSwitch(onRedTeam ? 1 : 2);

    }

    public void startLL() {
        limelight.start();
    }

    public void setLLPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
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

    public Pose getMegaTagPos() {
        LLResult llResult = limelight.getLatestResult();
        Pose botPose = null;
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose3D = llResult.getBotpose();
            botPose = new Pose(botPose3D.getPosition().x, botPose3D.getPosition().y,
                    botPose3D.getOrientation().getYaw());
        }
        return botPose;
    }

    // start the limelight - not done in constructor b/c limelight uses up energy
    public void startLimelight() {
        limelight.start();
    }
}
