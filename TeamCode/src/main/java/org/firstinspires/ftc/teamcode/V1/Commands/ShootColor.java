package org.firstinspires.ftc.teamcode.V1.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.V1.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V1.Subsystems.SpindexerSubsystem;

public class ShootColor extends CommandBase {
    private OuttakeSubsystem outtake;
    private SpindexerSubsystem spindexer;
    private ElapsedTime timer;
    String color;

    public ShootColor(OuttakeSubsystem outtakeSub, SpindexerSubsystem spindexSub, String color) {
        outtake = outtakeSub;
        spindexer = spindexSub;
        timer = new ElapsedTime();
        this.color = color;
    }

    @Override
    public void initialize() {
        if(outtake.getFlywheelSpeed() < outtake.getTargetSpeed() - 5
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

    @Override
    public void execute() {
        if(timer.milliseconds() > 1000) {
            outtake.kickUp();
        }
    }

    @Override
    public void end(boolean interrupted) {
        outtake.resetKicker();
    }

    @Override
    public boolean isFinished() {
        return outtake.getKickerPos() > 0.48;
    }
}
