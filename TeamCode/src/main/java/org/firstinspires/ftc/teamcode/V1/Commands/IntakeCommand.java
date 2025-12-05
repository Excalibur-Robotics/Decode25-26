package org.firstinspires.ftc.teamcode.V1.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.V1.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V1.Subsystems.SpindexerSubsystem;

public class IntakeCommand extends CommandBase {
    private IntakeSubsystem intake;
    private SpindexerSubsystem spindexer;

    public IntakeCommand(IntakeSubsystem intakeSub, SpindexerSubsystem spindexSub) {
        intake = intakeSub;
        spindexer = spindexSub;

        addRequirements(intake, spindexer);
    }

    @Override
    public void initialize() {
        if(spindexer.getNumArtifacts() == 3)
            cancel();
        intake.activateIntake();
    }

    @Override
    public void execute() {
        /* Need to implement color sensor
        if(spindexer.detectsArtifact()) {
            spindexer.addArtifact(spindexer.getColor());
            spindexer.rotateCCW();
        }
         */
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return spindexer.getNumArtifacts() == 3;
    }
}
