package org.firstinspires.ftc.teamcode.V2.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

import java.util.ArrayList;

public class CheckSpindexer extends CommandBase {
    SpindexerSubsystem spindexer;
    int count = 0;

    public CheckSpindexer(SpindexerSubsystem spindexer) {
        this.spindexer = spindexer;
    }

    @Override
    public void initialize() {
        if(spindexer.inOuttakeMode())
            spindexer.setToIntakeMode();
    }

    @Override
    public void execute() {
        ArrayList<String> indexer = spindexer.getIndexerState();
        spindexer.powerSpindexer();
        if(spindexer.getSpindexerPower() == 0) {
            if(spindexer.getGreenPixels() > 90000) {

            }
        }
    }
}
