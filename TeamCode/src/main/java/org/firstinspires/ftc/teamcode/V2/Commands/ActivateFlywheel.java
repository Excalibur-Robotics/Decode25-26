package org.firstinspires.ftc.teamcode.V2.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.V2.LHV2PID;
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
    Gamepad gamepad;
    Gamepad.RumbleEffect rumble;
    boolean hasRumbled;

    LHV2PID flywheelPID;
    // constants need tuning
    public static double kP = 0.015;
    public static double kI = 0;
    public static double kD = 0;
    public static double kV = 0.000795; // feedforward constant

    public double flywheelSpeed; // in rpm

    public ActivateFlywheel(OuttakeSubsystem outtakeSub, Gamepad gamepad) {
        outtake = outtakeSub;
        this.gamepad = gamepad;
        rumble = new Gamepad.RumbleEffect.Builder().addStep(1, 1, 500).build();
    }

    public ActivateFlywheel(OuttakeSubsystem outtakeSub) {
        outtake = outtakeSub;
        gamepad = null;
    }

    @Override
    public void initialize() {
        hasRumbled = false;
        flywheelPID = new LHV2PID(kP, kI, kD, kV);
    }

    @Override
    public void execute() {
        double power = 0;
        if(gamepad == null || gamepad.right_trigger > 0.5) {
            power = flywheelPID.Calculate(outtake.getTargetSpeed(), outtake.getFlywheelSpeed());
        }
        else {
            power = flywheelPID.Calculate(0, outtake.getFlywheelSpeed());
        }
        outtake.setFlywheelPower(power);
        if(gamepad != null && outtake.atTargetSpeed() && !hasRumbled) {
            gamepad.runRumbleEffect(rumble);
            hasRumbled = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        outtake.setFlywheelPower(0);
    }

    @Override
    public boolean isFinished() {
        if(gamepad != null)
            return gamepad.right_trigger == 0 && Math.abs(outtake.getFlywheelSpeed()) < 10;
        else
            return false;
    }
}
