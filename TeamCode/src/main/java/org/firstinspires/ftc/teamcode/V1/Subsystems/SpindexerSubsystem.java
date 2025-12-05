package org.firstinspires.ftc.teamcode.V1.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class SpindexerSubsystem extends SubsystemBase {
    public DcMotor spindexMotor;
    // color sensor here

    private final int ticksPerRev = (int) spindexMotor.getMotorType().getTicksPerRev();
    private ArrayList<String> indexer;
    private int numArtifacts;

    public SpindexerSubsystem(HardwareMap hwMap) {
        spindexMotor = hwMap.get(DcMotor.class, "spindexer");
        // color sensor

        spindexMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        indexer = new ArrayList<String>();
        indexer.add("empty");
        indexer.add("empty");
        indexer.add("empty");
        numArtifacts = 0;
    }

    // rotate the spindexer an specified angle in radians
    private void rotateAngle(int angle) {
        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexMotor.setTargetPosition(ticksPerRev * angle / 360);
        spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexMotor.setPower(0.5);
    }

    public void rotateCW() {
        rotateAngle(-120);
        indexer.add(indexer.remove(0));
    }

    public void rotateCCW() {
        rotateAngle(120);
        indexer.add(0, indexer.remove(2));
    }

    public void setToOuttakeMode() {
        rotateAngle(30);
    }

    public void setToIntakeMode() {
        rotateAngle(-30);
    }

    public ArrayList<String> getIndexerState() {
        return indexer;
    }

    public int getNumArtifacts() {
        return numArtifacts;
    }

    public void addArtifact(String color) {
        indexer.set(0, color);
        numArtifacts++;
    }

    public void removeArtifact() {
        indexer.set(2, "empty");
        numArtifacts--;
    }
}
