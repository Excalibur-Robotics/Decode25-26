package org.firstinspires.ftc.teamcode.V2.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

public class ShootAll extends CommandBase {
    OuttakeSubsystem outtake;
    SpindexerSubsystem spindexer;
    private int count;
    private String[] motif = null;

    public ShootAll(OuttakeSubsystem outtake, SpindexerSubsystem spindexer) {
        this.outtake = outtake;
        this.spindexer = spindexer;
    }
    public ShootAll(OuttakeSubsystem outtake, SpindexerSubsystem spindexer, int id) {
        this.outtake = outtake;
        this.spindexer = spindexer;
        count = 0;
        motif = new String[3];
        if(id == 21) {
            motif[0] = "green";
            motif[1] = "purple";
            motif[2] = "purple";
        }
        if(id == 22) {
            motif[0] = "purple";
            motif[1] = "green";
            motif[2] = "purple";
        }
        if(id == 23) {
            motif[0] = "purple";
            motif[1] = "purple";
            motif[2] = "green";
        }

    }

    @Override
    public void execute() {
        if(motif != null) {
            if (spindexer.getIndexerState().contains(motif[count]))
                new ShootColor(outtake, spindexer, motif[count]).schedule(false);
            else
                new ShootArtifact(outtake, spindexer).schedule(false);
        }
        else {
            new ShootArtifact(outtake, spindexer).schedule(false);
        }
    }

    @Override
    public boolean isFinished() {
        return spindexer.getNumArtifacts() == 0;
    }
}
