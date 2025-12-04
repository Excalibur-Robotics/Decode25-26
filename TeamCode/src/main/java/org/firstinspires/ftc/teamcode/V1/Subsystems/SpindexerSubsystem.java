package org.firstinspires.ftc.teamcode.V1.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
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

        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        indexer = new ArrayList<String>();
        indexer.add("empty");
        indexer.add("empty");
        indexer.add("empty");
        numArtifacts = 0;
    }

    public void goToPosition(int pos) {
        spindexMotor.setTargetPosition(pos);
        spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexMotor.setPower(0.5);
    }

    public void nextPosition() {
        int currentPos = spindexMotor.getCurrentPosition();
        goToPosition((int) (currentPos + ticksPerRev / 3));
    }

    public ArrayList<String> getIndexerState() {
        return indexer;
    }

    public int getNumArtifacts() {
        return numArtifacts;
    }

    public void addArtifact(int pos, String color) {
        indexer.set(pos, color);
        numArtifacts++;
    }

    public void removeArtifact(int pos) {
        indexer.set(pos, "empty");
        numArtifacts--;
    }

    public int ticksToPos(int posInTicks) {
        return (posInTicks + 10) * 3 / ticksPerRev;
    }
}
