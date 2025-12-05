package org.firstinspires.ftc.teamcode.V1.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.V1.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V1.Subsystems.SpindexerSubsystem;

/*
This is the command to spin up the flywheel. The flywheel speed is constant,
and is stored in the variable flywheelSpeed. At the start of the command, the
flywheel is set to this speed. If the spindexer is in intake mode, it is set to
outtake mode. At the end of the command, the flywheel speed is set to 0.
 */

public class ActivateFlywheel extends CommandBase {
    private OuttakeSubsystem outtake;
    private SpindexerSubsystem spindexer;

    private static final double flywheelSpeed = 1000;

    public ActivateFlywheel(OuttakeSubsystem outtakeSub, SpindexerSubsystem spindexSub) {
        outtake = outtakeSub;
        spindexer = spindexSub;

        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        outtake.setFlywheelSpeed(flywheelSpeed);
        if(!spindexer.inOuttakeMode())
            spindexer.setToOuttakeMode();

    }

    @Override
    public void end(boolean interrupted) {
        outtake.setFlywheelSpeed(0);
    }
}
