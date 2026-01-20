package org.firstinspires.ftc.teamcode.V2.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;

/*
This is the command to aim the turret towards the apriltag on the goal.
It uses the tx value from the limelight and a PID controller to control the
power of the turret motor so that it faces the apriltag. It is used in teleop
as a default command so that the turret always faces the goal during teleop.
 */

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
        // start limelight when the command is scheduled
        outtake.startLimelight();
    }

    @Override
    public void execute() {
        // read limelight data and get tx, which is the
        // horizontal angle of the apriltag from the center
        LLResult llData = outtake.readLimelight();
        double tx = 0;
        if(llData != null && llData.isValid()) {
            tx = llData.getTx();
        }
        // pass tx to the pid controller to calculate motor power
        double power = turretController.calculate(tx);
        outtake.powerTurret(power);
    }

    @Override
    public void end(boolean interrupted) {
        outtake.powerTurret(0);
    }
}
