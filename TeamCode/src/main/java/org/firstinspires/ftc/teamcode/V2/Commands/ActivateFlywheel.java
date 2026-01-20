package org.firstinspires.ftc.teamcode.V2.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

/*
This is the command to spin up the flywheel. The flywheel speed is constant,
and is stored in the variable flywheelSpeed. At the start of the command, the
flywheel is set to this speed. If the spindexer is in intake mode, it is set to
outtake mode. At the end of the command, the flywheel speed is set to 0.
 */

@Config
public class ActivateFlywheel extends CommandBase {
    private OuttakeSubsystem outtake;
    private SpindexerSubsystem spindexer;
    Gamepad gamepad;
    Gamepad.RumbleEffect rumble;
    boolean hasRumbled;

    PIDController flywheelPID;
    // constants need tuning
    public static double kP = 0.05;
    public static double kI = 0;
    public static double kD = 0;
    public static double flywheelSpeed = 1000; // in rpm

    public ActivateFlywheel(OuttakeSubsystem outtakeSub, SpindexerSubsystem spindexSub, Gamepad gamepad) {
        outtake = outtakeSub;
        spindexer = spindexSub;
        this.gamepad = gamepad;
        rumble = new Gamepad.RumbleEffect.Builder().addStep(1, 1, 500).build();
        flywheelPID = new PIDController(kP, kI, kD);
        flywheelPID.setSetPoint(flywheelSpeed);

        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        //outtake.setFlywheelSpeed(flywheelSpeed);
        if(!spindexer.inOuttakeMode())
            spindexer.setToOuttakeMode();
        hasRumbled = false;
    }

    @Override
    public void execute() {
        double power = flywheelPID.calculate(outtake.getFlywheelSpeed());
        outtake.setFlywheelPower(power);
        if(outtake.getFlywheelSpeed() > outtake.getTargetSpeed() - 5 && !hasRumbled) {
            gamepad.runRumbleEffect(rumble);
            hasRumbled = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        outtake.setFlywheelSpeed(0);
    }
}
