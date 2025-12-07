package org.firstinspires.ftc.teamcode.V1.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.ArrayList;

/*
Note: color sensor code still needs to be added

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

public class SpindexerSubsystem extends SubsystemBase {
    public DcMotor spindexMotor;
    public NormalizedColorSensor colorSensor;

    private int ticksPerRev;
    private ArrayList<String> indexer;
    private int numArtifacts;
    private boolean shootMode;

    public SpindexerSubsystem(HardwareMap hwMap) {
        spindexMotor = hwMap.get(DcMotor.class, "spindexer");
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");

        spindexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        indexer = new ArrayList<String>();
        indexer.add("empty");
        indexer.add("empty");
        indexer.add("empty");
        numArtifacts = 0;
        shootMode = false;
        ticksPerRev = (int) spindexMotor.getMotorType().getTicksPerRev();
    }

    // rotate the spindexer an specified angle in radians
    private void rotateAngle(int angle) {
        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexMotor.setTargetPosition(ticksPerRev * angle / 360);
        spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexMotor.setPower(0.05);
    }

    public double getSpindexerAngle() {
        return (double) spindexMotor.getCurrentPosition() / ticksPerRev * 360;
    }

    // rotate spindexer clockwise one slot (120 degrees)
    public void rotateCW() {
        rotateAngle(-120);
        indexer.add(indexer.remove(0));
    }

    // rotate spindexer counter clockwise one slot
    public void rotateCCW() {
        rotateAngle(120);
        indexer.add(0, indexer.remove(2));
    }

    // rotate 30 degrees to outtake mode
    public void setToOuttakeMode() {
        rotateAngle(30);
        shootMode = true;
    }

    // rotate back 30 degrees to intake mode
    public void setToIntakeMode() {
        rotateAngle(-30);
        shootMode = false;
    }

    public boolean inOuttakeMode() {
        return shootMode;
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
        NormalizedRGBA rgba = colorSensor.getNormalizedColors();
        double hue = JavaUtil.colorToHue(rgba.toColor());
        String color = "empty";
        if(hue >= 210 && hue <= 245) {
            color = "purple";
        }
        else if(hue >= 150 && hue <= 180) {
            color = "green";
        }
        return color;
    }

    public boolean detectsArtifact() {
        String color = getColor();
        return color.equals("purple") || color.equals("green");
    }
}
