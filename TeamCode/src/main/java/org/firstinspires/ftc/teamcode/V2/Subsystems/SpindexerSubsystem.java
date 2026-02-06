package org.firstinspires.ftc.teamcode.V2.Subsystems;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.V2.LHV2PID;

import java.util.ArrayList;

/*
This is the subsystem for the spindexer of V1. It includes a motor to turn the
spindexer and a color sensor.
spindexer motor: can rotate spindexer one slot and switch modes*
color sensor: detects when an artifact is in slot 0 and its color

This subsystem also contains an arraylist to store the state of the spindexer,
which is modified when the spindexer rotates and when an artifact is intaked or
outtaked. It also has a variable containing the number of artifacts currently
in the spindexer

*Because the outtake and intake positions don't line up on the robot, the
spindexer can be rotate 30 degrees to switch between intake and outtake mode.
When this happens, the state of the indexer arraylist doesn't change.
 */

@Config
public class SpindexerSubsystem extends SubsystemBase {
    public DcMotorEx spindexMotor;
    //public NormalizedColorSensor colorSensor;

    private ArrayList<String> indexer;
    private int numArtifacts;
    private boolean OuttakeMode;

    // spindexer pid constants
    public static double kP = 0.0006; //0.00012;
    public static double kI = 0; //0.0000000025;
    public static double kD = 0.008; //-0.0002;
    public static int tolerance = 35;
    public static int velocityTolerance = 15;
    public static int ticksPerRev = 8192; // cpm of bore encoder

    public LHV2PID PID;
    private double TP; // target position in ticks

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();

    public SpindexerSubsystem(HardwareMap hwMap) {
        spindexMotor = hwMap.get(DcMotorEx.class, "Bore");
        //colorSensor = hwMap.get(NormalizedColorSensor.class, "CS1");

        spindexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PID = new LHV2PID(kP, kI, kD);
        TP = 0;
        timer.reset();

        indexer = new ArrayList<String>();
        indexer.add("empty");
        indexer.add("empty");
        indexer.add("empty");
        numArtifacts = 3; // start with 3 preloads
        OuttakeMode = true; // start spindexer in outtake mode
    }

    // set target position of spindexer in ticks
    public void setTP(double tp) {
        TP = tp;
    }

    public double getTP() {
        return TP;
    }

    // power spindexer based on PID
    public void powerSpindexer() {
        double CP = spindexMotor.getCurrentPosition();
        if(abs((TP) - CP) > tolerance || abs(spindexMotor.getVelocity()) > velocityTolerance) {
            if(timer.milliseconds()>15) {
                CP = spindexMotor.getCurrentPosition();
                double MotorPower = -PID.Calculate(TP, CP);
                spindexMotor.setPower(MotorPower);
                timer.reset();
            }
        }
    }

    // returns angle spindexer has rotated in degrees
    public double getSpindexerAngle() {
        return (double) spindexMotor.getCurrentPosition() / ticksPerRev * 360;
    }

    // rotate spindexer clockwise one slot (120 degrees)
    public void rotateCW() {
        setTP(TP - 120.0 / 360 * ticksPerRev);
        indexer.add(indexer.remove(0));
    }

    // rotate spindexer counter-clockwise one slot
    public void rotateCCW() {
        setTP(TP + 120.0 / 360 * ticksPerRev);
        indexer.add(0, indexer.remove(2));
    }

    // rotate 60 degrees to outtake mode
    public void setToOuttakeMode() {
        setTP(TP - 60.0 / 360 * ticksPerRev);
        OuttakeMode = true;
    }

    // rotate back 60 degrees to intake mode
    public void setToIntakeMode() {
        setTP(TP + 60.0 / 360 * ticksPerRev);
        OuttakeMode = false;
    }

    public boolean inOuttakeMode() {
        return OuttakeMode;
    }

    // get the arraylist of the artifacts
    public ArrayList<String> getIndexerState() {
        return indexer;
    }

    // get the number of artifacts in the spindexer
    public int getNumArtifacts() {
        return numArtifacts;
    }

    // add an artifact to slot 0 when intaked
    public void addArtifact(String color) {
        if(indexer.get(0).equals("empty")) {
            indexer.set(0, color);
            numArtifacts++;
        }
    }

    // remove an artifact from slot 2 when outtaked
    public void removeArtifact() {
        if(!indexer.get(2).equals("empty")) {
            indexer.set(2, "empty");
            numArtifacts--;
        }
    }

    // get the color the color sensor currently sees.
    public String getColor() {
        /*NormalizedRGBA rgba = colorSensor.getNormalizedColors();
        double hue = JavaUtil.colorToHue(rgba.toColor());
        String color = "empty";
        if(hue >= 210 && hue <= 245) {
            color = "purple";
        }
        else if(hue >= 150 && hue <= 180) {
            color = "green";
        }

         */
        return "";//color;
    }

    public boolean detectsArtifact() {
        String color = getColor();
        return color.equals("purple") || color.equals("green");
    }


}
