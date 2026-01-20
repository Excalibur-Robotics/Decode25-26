package org.firstinspires.ftc.teamcode.V2.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.V2.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

/*
Simple TeleOp for V2 with the goal of just being able to launch artifacts.
Driver manually controls each individual movement, no automatic actions.
 */

@TeleOp
public class TurretTest extends CommandOpMode {
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    OuttakeSubsystem outtake;
    DrivetrainSubsystem drivetrain;

    //GamepadEx gamepadEx = new GamepadEx(gamepad1);

    private PIDController turretController;
    public static double kP = 0.05;
    public static double kD = 0.0;

    public static double hoodUp = 0.5;
    public static double hoodDown = 0.0;

    @Override
    public void initialize() {
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        drivetrain = new DrivetrainSubsystem(hardwareMap, gamepad1);

        turretController = new PIDController(kP, 0, kD);
        turretController.setSetPoint(0);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        // flywheel - right trigger
        if(gamepad1.right_trigger > 0.5) {
            outtake.setFlywheelPower(1);
        }
        else {
            outtake.setFlywheelPower(0);
        }

        // move hood up - dpad up
        if(gamepad1.dpad_up) {
            outtake.setHood(hoodUp);
        }

        // move hood down - dpad down
        if(gamepad1.dpad_down) {
            outtake.setHood(hoodDown);
        }

        // aim turret with limelight
        LLResult llData = outtake.readLimelight();
        double tx = 0;
        if (llData != null && llData.isValid()) {
            tx = llData.getTx();
        }
        double power = turretController.calculate(tx);
        outtake.powerTurret(power);


        //telemetry.addData("kicker position", outtake.getKickerPos());
        telemetry.addData("flywheel speed", outtake.getFlywheelSpeed());
        telemetry.addData("tx", tx);
        telemetry.update();
    }
}
