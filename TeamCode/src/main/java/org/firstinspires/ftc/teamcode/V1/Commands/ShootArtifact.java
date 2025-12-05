package org.firstinspires.ftc.teamcode.V1.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.V1.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V1.Subsystems.SpindexerSubsystem;

/*
This is the command to shoot an artifact of any color.
It rotates so that an artifact is in the shoot slot, then the kicker kicks
the artifact to the outtake.
 */

public class ShootArtifact extends CommandBase {
    private OuttakeSubsystem outtake;
    private SpindexerSubsystem spindexer;
    private ElapsedTime timer;

    public ShootArtifact(OuttakeSubsystem outtakeSub, SpindexerSubsystem spindexSub) {
        outtake = outtakeSub;
        spindexer = spindexSub;
        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        // don't allow command to run if the flywheel isn't up to speed
        // or if the spindexer is empty
        if(outtake.getFlywheelSpeed() < outtake.getTargetSpeed() - 5
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
        // uses a timer to make sure the spindexer has rotated fully before
        // activating the kicker
        if(timer.milliseconds() > 1000) {
            outtake.kickUp();
            spindexer.removeArtifact(); // remove artifact from indexer arraylist
        }
    }

    @Override
    public void end(boolean interrupted) {
        // the command ends by resetting the kicker
        outtake.resetKicker();
    }

    @Override
    public boolean isFinished() {
        // after kicking the artifact, the command will continue until the
        // kicker reaches the target position
        return outtake.getKickerPos() > 0.48;
    }
}
