package org.firstinspires.ftc.teamcode.V2.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    private LHV2PID turretController;
    public static double kP = 0.02;
    public static double kD = 0.0001;

    public static double hoodUp = 0.75;
    public static double hoodDown = 0.1;

    public static double flywheelPower = 0.8;

    @Override
    public void initialize() {
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        drivetrain = new DrivetrainSubsystem(hardwareMap, gamepad1);

        turretController = new LHV2PID(kP, 0, kD);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        /*
        // kicker - X
        if(gamepad1.x) {
            outtake.kickUp();
        }
        else if(outtake.getKickerPos() == outtake.kickerUp) {
            outtake.resetKicker();
        }
         */

        // intake - left trigger
        if(gamepad1.left_trigger > 0.5) {
            intake.activateIntake();
        }
        else {
            intake.stopIntake();
        }

        // flywheel - right trigger
        if(gamepad1.right_trigger > 0.5) {
            outtake.setFlywheelPower(flywheelPower);
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
        LLResult llData = outtake.readLimelight();
        double tx = 0;
        if (llData != null && llData.isValid()) {
            tx = llData.getTx();
        }
        double power = turretController.Calculate(0, tx);
        outtake.powerTurret(power);


        //telemetry.addData("kicker position", outtake.getKickerPos());
        //telemetry.addData("flywheel speed", outtake.getFlywheelSpeed());
        telemetry.addData("tx", tx);
        telemetry.addData("hood angle", outtake.getHoodAngle());
        telemetry.addData ("spindexer position", spindexer.getSpindexerAngle());
        /*ArrayList<String> indexer = spindexer.getIndexerState();
        telemetry.addData("indexer state", indexer.get(0) + " " +
                          indexer.get(1) + " " + indexer.get(2));
        */
        telemetry.update();
    }
}
