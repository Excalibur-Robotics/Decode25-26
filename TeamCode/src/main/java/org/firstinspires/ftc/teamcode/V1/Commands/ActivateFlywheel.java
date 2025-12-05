package org.firstinspires.ftc.teamcode.V1.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.V1.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V1.Subsystems.SpindexerSubsystem;

public class ActivateFlywheel extends CommandBase {
    private OuttakeSubsystem outtake;
    private SpindexerSubsystem spindexer;

    public ActivateFlywheel(OuttakeSubsystem outtakeSub, SpindexerSubsystem spindexSub) {
        outtake = outtakeSub;
        spindexer = spindexSub;

        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        outtake.setFlywheelSpeed(1000);
        spindexer.setToOuttakeMode();

    }

    @Override
    public void end(boolean interrupted) {
        outtake.setFlywheelSpeed(0);
        spindexer.setToIntakeMode();
    }
}
