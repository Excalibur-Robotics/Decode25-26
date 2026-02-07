package org.firstinspires.ftc.teamcode.V2.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

public class ShootColor extends ShootArtifact {
    private OuttakeSubsystem outtake;
    private SpindexerSubsystem spindexer;
    private ElapsedTime timer;
    private boolean artifactKickedUp;
    private String color;

    public ShootColor(OuttakeSubsystem outtakeSub, SpindexerSubsystem spindexSub, String color) {
        super(outtakeSub, spindexSub);
        this.color = color;
    }

    @Override
    public void initialize() {
        if(outtake.getFlywheelSpeed() < outtake.getTargetSpeed() - 20
                || spindexer.getNumArtifacts() == 0) {
            cancel();
        }
        if(!spindexer.getIndexerState().get(2).equals(color)) {
            if(spindexer.getIndexerState().get(1).equals(color)) {
                spindexer.rotateCCW();
            }
            else if(spindexer.getIndexerState().get(0).equals(color)) {
                spindexer.rotateCW();
            }
            else {
                cancel();
            }
            timer.reset();
        }
    }
}
