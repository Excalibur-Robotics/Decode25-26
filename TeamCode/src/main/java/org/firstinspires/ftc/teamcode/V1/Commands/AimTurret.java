package org.firstinspires.ftc.teamcode.V1.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.V1.Subsystems.OuttakeSubsystem;

@Configurable
public class AimTurret extends CommandBase {
    private OuttakeSubsystem outtake;
    private PIDController turretController;
    public static double kp = 0.05;
    public static double kd = 0.0;

    public AimTurret(OuttakeSubsystem outtakeSub) {
        outtake = outtakeSub;
        turretController = new PIDController(kp, 0, kd);
        turretController.setSetPoint(0);
    }

    @Override
    public void initialize() {
        outtake.startLimelight();
    }

    @Override
    public void execute() {
        LLResult llData = outtake.readLimelight();
        double tx = 0;
        if(llData != null && llData.isValid()) {
            tx = llData.getTx();
        }
        double power = turretController.calculate(tx);
        outtake.powerTurret(power);
    }

    @Override
    public void end(boolean interrupted) {
        outtake.powerTurret(0);
    }
}
