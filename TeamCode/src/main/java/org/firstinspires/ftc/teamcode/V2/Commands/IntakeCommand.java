package org.firstinspires.ftc.teamcode.V2.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

/*
This is the command to intake artifacts for V1. When the command starts, the
intake motor is activated. If the spindexer is in outtake mode, it is set to
intake mode. When the color sensor detects an artifact, it records its position
in the spindexer and rotates and spindexer counter-clockwise to the next slot.
The command ends when the spindexer is full or the driver releases the trigger.
 */

public class IntakeCommand extends CommandBase {
    private IntakeSubsystem intake;
    private SpindexerSubsystem spindexer;

    // if artifact was detected in the previous loop - used for edge detection
    private boolean artifactPreviouslyDetected;
    private boolean spindexing;

    public IntakeCommand(IntakeSubsystem intakeSub, SpindexerSubsystem spindexSub) {
        intake = intakeSub;
        spindexer = spindexSub;

        addRequirements(intake, spindexer);
    }

    @Override
    public void initialize() {
        // Don't allow command to run if spindexer is full
        if(spindexer.getNumArtifacts() == 3)
            this.cancel();
        if(spindexer.inOuttakeMode()) {
            spindexer.setToIntakeMode();
        }
        // activate intake at start of command
        intake.activateIntake();

        artifactPreviouslyDetected = true;
        spindexing = true;
    }

    @Override
    public void execute() {
        boolean artifactDetected = spindexer.detectsArtifact();
        if(!spindexer.isSpindexing()) {
            if(artifactDetected && !artifactPreviouslyDetected) {
                spindexer.addArtifact(spindexer.getColor());
                spindexer.rotateCCW();
                spindexing = true;
            }
        }
        /*else {
            if(spindexer.isSpindexing()) {
                spindexing = false;
            }
        }*/
        spindexer.powerSpindexer();
        artifactPreviouslyDetected = artifactDetected;
    }

    @Override
    public void end(boolean interrupted) {
        // when the command ends, stop the intake motor
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        // end the command if the spindexer is full
        return false; //spindexer.getNumArtifacts() == 3;
    }
}
