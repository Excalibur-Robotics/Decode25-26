package org.firstinspires.ftc.teamcode.V2.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

/*
This is the command to shoot an artifact of any color.
It rotates so that an artifact is in the shoot slot, then the kicker kicks
the artifact to the outtake.
 */

public class ShootArtifact extends CommandBase {
    private OuttakeSubsystem outtake;
    private SpindexerSubsystem spindexer;
    private ElapsedTime timer;

    private boolean artifactKickedUp;

    public ShootArtifact(OuttakeSubsystem outtakeSub, SpindexerSubsystem spindexSub) {
        outtake = outtakeSub;
        spindexer = spindexSub;
        timer = new ElapsedTime();

        artifactKickedUp = false;
    }

    @Override
    public void initialize() {
        // don't allow command to run if the flywheel isn't up to speed
        // or if the spindexer is empty
        if(outtake.getFlywheelSpeed() < outtake.getTargetSpeed() - 20
                || spindexer.getNumArtifacts() == 0) {
            cancel();
        }
        // logic to rotate to correct slot
        if(spindexer.getIndexerState().get(2).equals("empty")) {
            if(!spindexer.getIndexerState().get(1).equals("empty")) {
                spindexer.rotateCCW();
            }
            else {
                spindexer.rotateCW();
            }
            timer.reset();
        }
    }

    @Override
    public void execute() {
        spindexer.powerSpindexer();
        // uses a timer to make sure the spindexer has rotated fully before
        // activating the kicker
        if(timer.milliseconds() > 1000 && !artifactKickedUp) {
            outtake.kickUp();
            artifactKickedUp = true;
        }
        // once kicker is all the way up, it is reset
        if(outtake.getKickerPos() > outtake.getKickerUp() - 0.02) {
            outtake.resetKicker();
            spindexer.removeArtifact(); // remove artifact from indexer arraylist
        }
    }

    @Override
    public void end(boolean interrupted) {
        // the command ends by making sure the kicker is reset
        outtake.resetKicker();
    }

    @Override
    public boolean isFinished() {
        // ends the command when the kicker is reset
        return outtake.getKickerPos() < outtake.getKickerDown() + 0.02;
    }
}
