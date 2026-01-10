package org.firstinspires.ftc.teamcode.V1;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.V1.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.V1.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V1.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V1.Subsystems.SpindexerSubsystem;

/*
Simple TeleOp for V2 with the goal of just being able to launch artifacts.
Driver manually controls each individual movement, no automatic actions.
 */

public class V2SimpleTeleOp extends CommandOpMode {
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    OuttakeSubsystem outtake;
    DrivetrainSubsystem drivetrain;

    GamepadEx gamepadEx = new GamepadEx(gamepad1);

    private PIDController turretController;
    public static double kP = 0.05;
    public static double kD = 0.0;

    @Override
    public void initialize() {
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        drivetrain = new DrivetrainSubsystem(hardwareMap, gamepadEx);

        turretController = new PIDController(kP, 0, kD);
        turretController.setSetPoint(0);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        // kicker - X
        if(gamepad1.x) {
            outtake.kickUp();
        }
        else {
            outtake.resetKicker();
        }

        // intake - left trigger
        if(gamepad1.left_trigger > 0.5) {
            intake.activateIntake();
        }
        else {
            intake.stopIntake();
        }

        // flywheel - right trigger
        if(gamepad1.right_trigger > 0.5) {
            outtake.setFlywheelPower(1);
        }
        else {
            outtake.setFlywheelPower(0);
        }

        // rotate spindexer CCW - right bumper
        if(gamepad1.rightBumperWasPressed()) {
            spindexer.rotateCCW();
        }

        // rotate spindexer CW - left bumper
        if(gamepad1.leftBumperWasPressed()) {
            spindexer.rotateCW();
        }

        // rotate spindexer half a turn to switch between intake/outtake - Y
        if(gamepad1.yWasPressed()) {
            spindexer.setToOuttakeMode();
        }

        // move hood up - dpad up
        if(gamepad1.dpad_up) {
            outtake.setHood(0.5);
        }

        // move hood down - dpad down
        if(gamepad1.dpad_down) {
            outtake.setHood(0);
        }

        // aim turret with limelight - hold A
        if(gamepad1.a) {
            LLResult llData = outtake.readLimelight();
            double tx = 0;
            if (llData != null && llData.isValid()) {
                tx = llData.getTx();
            }
            double power = turretController.calculate(tx);
            outtake.powerTurret(power);
        }

        telemetry.addData("kicker position", outtake.getKickerPos());
        telemetry.addData("flywheel speed", outtake.getFlywheelSpeed());
        telemetry.addData ("spindexer position", spindexer.getSpindexerAngle());
        /*ArrayList<String> indexer = spindexer.getIndexerState();
        telemetry.addData("indexer state", indexer.get(0) + " " +
                          indexer.get(1) + " " + indexer.get(2));
        telemetry.update();*/
    }
}
