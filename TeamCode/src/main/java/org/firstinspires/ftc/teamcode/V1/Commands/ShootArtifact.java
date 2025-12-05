package org.firstinspires.ftc.teamcode.V1.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.V1.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V1.Subsystems.SpindexerSubsystem;

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
        if(outtake.getFlywheelSpeed() < outtake.getTargetSpeed() - 5
                || spindexer.getNumArtifacts() == 0) {
            cancel();
        }
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
        if(timer.milliseconds() > 1000) {
            outtake.kickUp();
        }
    }

    @Override
    public void end(boolean interrupted) {
        outtake.kickerDown();
    }

    @Override
    public boolean isFinished() {
        return outtake.getKickerPos() > 0.48;
    }
}
