package org.firstinspires.ftc.teamcode.V2.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

public class ShootAll extends CommandBase {
    OuttakeSubsystem outtake;
    SpindexerSubsystem spindexer;

    public ShootAll(OuttakeSubsystem outtake, SpindexerSubsystem spindexer) {
        this.outtake = outtake;
        this.spindexer = spindexer;
    }

    @Override
    public void execute() {
        new ShootArtifact(outtake, spindexer).schedule(false);
    }

    @Override
    public boolean isFinished() {
        return spindexer.getNumArtifacts() == 0;
    }
}
