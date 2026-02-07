package org.firstinspires.ftc.teamcode.V2.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.V2.Commands.ActivateFlywheel;
import org.firstinspires.ftc.teamcode.V2.LHV2PID;
import org.firstinspires.ftc.teamcode.V2.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

import java.util.ArrayList;

/*
Simple TeleOp for V2 with the goal of just being able to launch artifacts.
Driver manually controls each individual movement, no automatic actions.
 */
@Config
@TeleOp (name = "BasicTeleOpV2")
public class V2SimpleTeleOp extends OpMode {
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    OuttakeSubsystem outtake;
    DrivetrainSubsystem drivetrain;

    public static double hoodUp = 0.45;
    public static double hoodDown = 0;

    private ElapsedTime kickerTimer;
    private boolean onRedTeam = true;

    ActivateFlywheel activateFlywheel;

    @Override
    public void init() {
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        drivetrain = new DrivetrainSubsystem(hardwareMap);

        activateFlywheel = new ActivateFlywheel(outtake, spindexer, gamepad1);

        outtake.setHood(hoodDown);
        kickerTimer = new ElapsedTime();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Choose Team Color","press X if blue team, B if red team");
        if(gamepad1.x) {
            outtake.setTeam(false);
            onRedTeam = false;
        }
        if(gamepad1.b) {
            outtake.setTeam(true);
            onRedTeam = true;
        }
        telemetry.addData("Team Color", onRedTeam ? "RED" : "BLUE");
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        drivetrain.teleOpDrive(gamepad1);
        spindexer.powerSpindexer(); // spindexer pid
        outtake.calculateFlywheelSpeed();
        outtake.calculateTurret(outtake.getTX()); // turret aim with apriltag

        // kicker - X
        if(gamepad1.x) {
            outtake.kickUp();
            kickerTimer.reset();
        }
        else if(outtake.getKickerPos() > outtake.getKickerDown() &&
                kickerTimer.milliseconds() > outtake.getTransferTime()) {
            outtake.resetKicker();
        }

        // intake - left trigger
        if(gamepad1.left_trigger > 0.5) {
            intake.activateIntake();
        }
        else if(gamepad1.a || gamepad1.dpad_left) {
            intake.setIntakePower(-1);
        }
        else {
            intake.stopIntake();
        }

        // flywheel - right trigger
        if(gamepad1.right_trigger > 0.5) {
            activateFlywheel.schedule();
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
            if(spindexer.inOuttakeMode())
                spindexer.setToIntakeMode();
            else
                spindexer.setToOuttakeMode();
        }

        // move hood up - dpad up
        if(gamepad1.dpad_up) {
            outtake.setHood(outtake.getHoodAngle()+0.05);
        }

        // move hood down - dpad down
        if(gamepad1.dpad_down) {
            outtake.setHood(hoodDown);
        }



        telemetry.addData("team color", onRedTeam ? "RED" : "BLUE");
        telemetry.addData("flywheel speed", outtake.getFlywheelSpeed()); // in rpm
        telemetry.addData("target speed", outtake.getTargetSpeed());
        telemetry.addData("flywheel power", outtake.flywheel.getPower());
        telemetry.addData("kicker position", outtake.getKickerPos());
        telemetry.addData("hood angle", outtake.getHoodAngle());
        telemetry.addData("turret position", outtake.getTurretPos());
        telemetry.addData("tx", outtake.getTX());
        telemetry.addData("apriltag ID", outtake.getApriltagID());
        telemetry.addData("spindexer position", spindexer.getSpindexerAngle());
        telemetry.addData("spindexer mode", spindexer.inOuttakeMode() ? "outtake" : "intake");
        telemetry.addData("target position", spindexer.getTP());
        ArrayList<String> indexer = spindexer.getIndexerState();
        telemetry.addData("indexer state", indexer.get(0) + " " +
                          indexer.get(1) + " " + indexer.get(2));
        telemetry.addData("spindexer" , spindexer.inOuttakeMode() ? "  " +
                indexer.get(2).charAt(0) : " " + indexer.get(2).charAt(0) + " " + indexer.get(1).charAt(0));
        telemetry.addData("state        ", spindexer.inOuttakeMode() ? " " + indexer.get(0).charAt(0)
                + " " + indexer.get(0).charAt(0) : "  " + indexer.get(0).charAt(0));

        telemetry.update();
    }
}
