package org.firstinspires.ftc.teamcode.V1.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.V1.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.V1.Subsystems.OuttakeSubsystem;

public class AimRobot extends CommandBase {
    private OuttakeSubsystem outtake;
    private DrivetrainSubsystem drivetrain;
    private PIDController headingController;
    public static double kp = 0.05;
    public static double kd = 0.0;

    public AimRobot(OuttakeSubsystem outtakeSub, DrivetrainSubsystem drivetrainSub) {
        outtake = outtakeSub;
        drivetrain = drivetrainSub;
        headingController = new PIDController(kp, 0, kd);
        headingController.setSetPoint(0);
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
        double power = headingController.calculate(tx);
        drivetrain.moveRobot(0, 0, power);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.moveRobot(0,0,0);
    }
}
