package org.firstinspires.ftc.teamcode.V2.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    public static int testFlywheelSpeed = 575;
    public static double testHoodAngle = 0.9;

    public static double kickerDist = 1.0; // difference of up and down position
    public static double kickerDown = 0.0; // kicker servo down position
    public static double transferTime = 600; // in milliseconds

    public static int turretTicksPerRev = 2151;
    private LHV2PID turretPID;
    public static double kP = 0.018; // needs to be tuned
    public static double kI = 0.0;
    public static double kD = 0.5; // needs to be tuned

    private boolean onRedTeam;

    public OuttakeSubsystem(HardwareMap hwMap) {
        flywheel = hwMap.get(DcMotorEx.class, "flywheel");
        turret = hwMap.get(DcMotor.class, "turret");
        hoodR = hwMap.get(Servo.class, "hoodR");
        hoodL = hwMap.get(Servo.class, "hoodL");
        kicker = hwMap.get(Servo.class, "kicker");
        limelight = hwMap.get(Limelight3A.class, "limelight");

        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        limelight.pipelineSwitch(1);
        hoodL.setDirection(Servo.Direction.FORWARD);
        hoodR.setDirection(Servo.Direction.REVERSE);
        kicker.setDirection(Servo.Direction.FORWARD);

        turretPID = new LHV2PID(kP, kI, kD);
        kicker.setPosition(kickerDown);
        //setHood(hoodPosClose);
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
        setTargetSpeed(testFlywheelSpeed);
        setHood(testHoodAngle);
    }

    public void calculateHood(Pose botPose) {
        InterpLUT hoodLUT = new InterpLUT();
        hoodLUT.add(-1, 0.15);
        hoodLUT.add(25, 0.15);
        hoodLUT.add(40, 0.4);
        hoodLUT.add(55, 0.7);
        hoodLUT.add(70, 0.9);
        hoodLUT.add(85, 1);
        hoodLUT.add(100, 1);
        hoodLUT.add(111, 1);
        hoodLUT.add(120, 1);
        hoodLUT.add(128, 1);
        hoodLUT.add(135, 1);
        hoodLUT.add(142, 1);
        hoodLUT.add(149, 1);
        hoodLUT.add(160, 1);
        hoodLUT.add(1000, 1);
        hoodLUT.createLUT();

        setHood(hoodLUT.get(distFromGoal(botPose)));
    }
    public void calculateFlywheel(Pose botPose) {
        InterpLUT fwLUT = new InterpLUT();
        fwLUT.add(-1, 500);
        fwLUT.add(25, 500);
        fwLUT.add(40, 475);
        fwLUT.add(55, 515);
        fwLUT.add(70, 545);
        fwLUT.add(85, 580);
        fwLUT.add(100,625);
        fwLUT.add(111,640);
        fwLUT.add(120,665);
        fwLUT.add(128,685);
        fwLUT.add(135,700);
        fwLUT.add(142,725);
        fwLUT.add(149,735);
        fwLUT.add(160,765);
        fwLUT.add(1000,820);
        fwLUT.createLUT();

        targetSpeed = (int) (fwLUT.get(distFromGoal(botPose)));
    }

    public double distFromGoal(Pose botPose) {
        double dist = 0;
        if(onRedTeam) {
            dist = Math.sqrt(Math.pow(144.0-botPose.getX(), 2) + Math.pow(144.0-botPose.getY(), 2));
        }
        else {
            dist = Math.sqrt(Math.pow(botPose.getX(), 2) + Math.pow(144.0-botPose.getY(), 2));
        }
        return dist;
    }

    // get flywheel speed in rpm
    public double getFlywheelSpeed() {
        return flywheel.getVelocity() * 60.0 / fwTicksPerRev;
    }

    // get the speed the flywheel should be spinning at
    public double getTargetSpeed() {
        return targetSpeed;
    }

    public boolean atTargetSpeed() {
        return getFlywheelSpeed() > targetSpeed - 20;
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
        if(getTA() > 0) {
            turret.setPower(turretPID.Calculate(0, CP));
        }
        else {
            turret.setPower(0);
        }
    }

    // input target angle
    public void rotateTurret(double angle) {
        if(angle > 230) {
            angle = 230;
        }
        else if(angle < -130) {
            angle = -130;
        }
        turret.setPower(turretPID.Calculate(angle, getTurretPos()));
    }

    // aim turret with robot position
    public void aimTurret(Pose botPose) {
        Pose goal = new Pose(onRedTeam ? 136 : 8, 142);
        double angle = Math.toDegrees(Math.atan((goal.getY() - botPose.getY()) / (goal.getX() - botPose.getX())));
        if(angle < 0)
            angle += 180;
        double botHeading = Math.toDegrees(botPose.getHeading());
        if(!onRedTeam && botHeading < -90)
            botHeading += 360;
        double turretAngle = angle - botHeading;
        rotateTurret(turretAngle);
    }

    public void scanMotif(Pose botPose) {
        Pose goal = new Pose(72, 144);
        double angle = Math.toDegrees(Math.atan((goal.getY() - botPose.getY()) / (goal.getX() - botPose.getX())));
        if(angle < 0)
            angle += 180;
        double botHeading = Math.toDegrees(botPose.getHeading());
        if(!onRedTeam && botHeading < -90)
            botHeading += 360;
        double turretAngle = angle - botHeading;
        rotateTurret(turretAngle);
    }

    /*public int scanMotif(Pose botPose) {
        int id = 0;
        double angle = Math.toDegrees(Math.atan((144 - botPose.getY()) / (72 - botPose.getX())));
        if(angle < 0)
            angle += 180;
        double botHeading = Math.toDegrees(botPose.getHeading());
        if(!onRedTeam && botHeading < -90)
            botHeading += 360;
        double turretAngle = angle - botHeading;
        rotateTurret(turretAngle);
        if(getApriltagID() > 20 && getApriltagID() < 24) {
            id = getApriltagID();
        }
        return id;
    }*/

    public void resetTurretEncoder() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // turret angle in degrees, straight forward is 0
    public double getTurretPos() {
        return turret.getCurrentPosition() * 11.93/7 * 360.0 / turretTicksPerRev;
    }

    // set the position of the hood
    public void setHood(double angle) {
        hoodR.setPosition(angle);
        hoodL.setPosition(angle);
    }

    public double getHoodAngle() {
        return hoodL.getPosition();
    }
    public double getTestHoodAngle() {
        return testHoodAngle;
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
    // position of green ball in motif (1, 2, or 3)
    // -1 if apriltag not seen
    public int getMotif() {
        int motif = -1;
        if(getApriltagID() > 20 && getApriltagID() < 24) {
            return getApriltagID() - 21;
        }
        return motif;
    }

    public Pose getMegaTagPos() {
        LLResult llResult = limelight.getLatestResult();
        Pose botPose = null;
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose3D = llResult.getBotpose();
            double x = botPose3D.getPosition().y * 3.28 * 12 + 72;
            double y = -botPose3D.getPosition().x * 3.28 * 12 + 72;
            double heading = (botPose3D.getOrientation().getYaw(AngleUnit.RADIANS) - Math.PI/2)
                              - Math.toRadians(getTurretPos());
            botPose = new Pose(x, y, heading);
        }
        else {
            botPose = new Pose(0, 0, 0);
        }
        return botPose;
    }
}
