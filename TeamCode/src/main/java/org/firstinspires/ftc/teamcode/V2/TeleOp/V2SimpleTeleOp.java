package org.firstinspires.ftc.teamcode.V2.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.V2.Commands.ActivateFlywheel;
import org.firstinspires.ftc.teamcode.V2.LHV2PID;
import org.firstinspires.ftc.teamcode.V2.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

/*
Simple TeleOp for V2 with the goal of just being able to launch artifacts.
Driver manually controls each individual movement, no automatic actions.
 */
@Config
@TeleOp (name = "BasicTeleOpV2")
public class V2SimpleTeleOp extends CommandOpMode {
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    OuttakeSubsystem outtake;
    DrivetrainSubsystem drivetrain;

    public static double hoodUp = 0.45;
    public static double hoodDown = 0;

    public static int flywheelSpeed= 600;

    ActivateFlywheel activateFlywheel;
    ActivateFlywheel deactivateFlywheel;

    @Override
    public void initialize() {
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        drivetrain = new DrivetrainSubsystem(hardwareMap, gamepad1);

        activateFlywheel = new ActivateFlywheel(outtake, spindexer, flywheelSpeed, gamepad1);
        deactivateFlywheel = new ActivateFlywheel(outtake, spindexer, 0, gamepad1);

        outtake.setHood(hoodDown);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        // kicker - X
        if(gamepad1.x) {
            outtake.kickUp();
        }
        else if (gamepad1.b){//if(outtake.getKickerPos() == outtake.getKickerUp()) {
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
            //outtake.setFlywheelPower(flywheelPower);
            activateFlywheel.schedule();
        }
        else if(activateFlywheel.isScheduled()){
            //outtake.setFlywheelPower(0);
            deactivateFlywheel.schedule();
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

        spindexer.powerSpindexer();

        // move hood up - dpad up
        if(gamepad1.dpad_up) {
            outtake.setHood(hoodUp);
        }

        // move hood down - dpad down
        if(gamepad1.dpad_down) {
            outtake.setHood(hoodDown);
        }

        // aim turret with limelight
        double tx = outtake.getTX();
        outtake.calculateTurret(tx);



        telemetry.addData("flywheel speed", outtake.getFlywheelSpeed()); // in rpm
        telemetry.addData("target speed", activateFlywheel.flywheelSpeed);
        telemetry.addData("flywheel power", outtake.flywheel.getPower());
        telemetry.addData("kicker position", outtake.getKickerPos());
        telemetry.addData("hood angle", outtake.getHoodAngle());
        telemetry.addData("turret position", outtake.getTurretPos());
        telemetry.addData("tx", tx);
        telemetry.addData("apriltag ID", outtake.getApriltagID());
        telemetry.addData ("spindexer position", spindexer.getSpindexerAngle());
        telemetry.addData("spindexer mode", spindexer.inOuttakeMode() ? "outtake" : "intake");
        telemetry.addData("target position", spindexer.getTP());
        /*ArrayList<String> indexer = spindexer.getIndexerState();
        telemetry.addData("indexer state", indexer.get(0) + " " +
                          indexer.get(1) + " " + indexer.get(2));
        */
        telemetry.update();
    }
}
