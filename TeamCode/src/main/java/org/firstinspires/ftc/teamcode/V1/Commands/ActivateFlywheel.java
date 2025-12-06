package org.firstinspires.ftc.teamcode.V1.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

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
    Gamepad gamepad;
    Gamepad.RumbleEffect rumble;
    boolean hasRumbled;

    private static final double flywheelSpeed = 1000;

    public ActivateFlywheel(OuttakeSubsystem outtakeSub, SpindexerSubsystem spindexSub, Gamepad gamepad) {
        outtake = outtakeSub;
        spindexer = spindexSub;
        this.gamepad = gamepad;
        rumble = new Gamepad.RumbleEffect.Builder().addStep(1, 1, 500).build();

        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        //outtake.setFlywheelSpeed(flywheelSpeed);
        outtake.setFlywheelPower(1);
        // no longer needed with new kicker position
        //if(!spindexer.inOuttakeMode())
        //    spindexer.setToOuttakeMode();
        hasRumbled = false;
    }

    @Override
    public void execute() {
        /*
        if(outtake.getFlywheelSpeed() > outtake.getTargetSpeed() - 5 && !hasRumbled) {
            gamepad.runRumbleEffect(rumble);
            hasRumbled = true;
        }
         */
    }

    @Override
    public void end(boolean interrupted) {
        //outtake.setFlywheelSpeed(0);
        outtake.setFlywheelPower(0);
    }
}
