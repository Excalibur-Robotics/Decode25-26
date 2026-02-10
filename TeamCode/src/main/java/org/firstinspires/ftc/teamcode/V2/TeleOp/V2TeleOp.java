package org.firstinspires.ftc.teamcode.V2.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.V2.Commands.ActivateFlywheel;
import org.firstinspires.ftc.teamcode.V2.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootArtifact;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootColor;
import org.firstinspires.ftc.teamcode.V2.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

import java.util.ArrayList;

/*
This is the OpMode for our TeleOp for V1. It uses a command based system with
FTCLib. The robot can intake artifacts, which are detected by a color sensor
and stored in the spindexer, and launch artifacts of a specified color by
automatically rotating the spindexer to the correct position
 */

@TeleOp
public class V2TeleOp extends CommandOpMode {
    // subsystems
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    OuttakeSubsystem outtake;
    DrivetrainSubsystem drivetrain;

    // commands
    IntakeCommand intakeCommand;
    ActivateFlywheel activateFlywheel;
    ShootArtifact shootArtifact;
    ShootColor shootPurple;
    ShootColor shootGreen;

    // gamepads
    GamepadEx gp1;
    GamepadEx gp2;

    // buttons/triggers
    Trigger leftTrigger;
    Trigger rightTrigger;
    Button rightBumper;
    Button X;
    Button B;
    Button A;

    private boolean onRedTeam = true;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        // initialize gamepads
        gp1 = new GamepadEx(gamepad1); // driving
        gp2 = new GamepadEx(gamepad2); // intake/outtake

        // initialize subsystems
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        drivetrain = new DrivetrainSubsystem(hardwareMap);

        // initialize commands
        intakeCommand = new IntakeCommand(intake, spindexer);
        activateFlywheel = new ActivateFlywheel(outtake, spindexer, gamepad1);
        shootArtifact = new ShootArtifact(outtake, spindexer);
        shootGreen = new ShootColor(outtake, spindexer, "green");
        shootPurple = new ShootColor(outtake, spindexer, "purple");

        // set buttons/triggers
        leftTrigger = new Trigger(() -> gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        rightTrigger = new Trigger(() -> gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        rightBumper = new GamepadButton(gp1, GamepadKeys.Button.RIGHT_BUMPER);
        X = new GamepadButton(gp1, GamepadKeys.Button.X);
        B = new GamepadButton(gp1, GamepadKeys.Button.B);
        A = new GamepadButton(gp1, GamepadKeys.Button.A);

        // Bind buttons/triggers with commands
        leftTrigger.whileActiveOnce(intakeCommand);
        rightTrigger.whenActive(activateFlywheel);
        X.whenPressed(shootArtifact);
        B.whenPressed(shootPurple);
        A.whenPressed(shootGreen);

        telemetry.addData("spindexer position", spindexer.getSpindexerAngle());
        telemetry.addData("spindexer target position", spindexer.getTargetAngle());
        telemetry.update();

        outtake.startLL();
/*
        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("Choose Team Color", "press X if blue team, B if red team");
            if (gamepad1.x) {
                outtake.setTeam(false);
                onRedTeam = false;
            }
            if (gamepad1.b) {
                outtake.setTeam(true);
                onRedTeam = true;
            }
            telemetry.addData("Team Color", onRedTeam ? "RED" : "BLUE");
        }

 */
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        drivetrain.teleOpDrive(gamepad1);
        outtake.calculateLaunch();
        outtake.calculateTurret(outtake.getTX());

        /*// move hood up - dpad up
        if(gamepad1.dpad_up) {
            outtake.setHood(outtake.getHoodFar());
        }

        // move hood down - dpad down
        if(gamepad1.dpad_down) {
            outtake.setHood(outtake.getHoodClose());
        }

         */

        if(gamepad1.rightBumperWasPressed()) {
            spindexer.rotateCCW();
        }
        if(gamepad1.leftBumperWasPressed()) {
            spindexer.rotateCW();
        }
        if(gamepad1.yWasPressed()) {
            if(spindexer.inOuttakeMode())
                spindexer.setToIntakeMode();
            else
                spindexer.setToOuttakeMode();
        }
        spindexer.powerSpindexer();



        telemetry.addData("flywheel command scheduled", activateFlywheel.isScheduled());
        telemetry.addData("shoot command scheduled", shootArtifact.isScheduled());
        telemetry.addData("intake command scheduled", intakeCommand.isScheduled());
        telemetry.addData("purple pixels", spindexer.getPurplePixels());
        telemetry.addData("green pixels", spindexer.getGreenPixels());
        telemetry.addData("Artifact in intake slot", spindexer.detectsArtifact());
        FtcDashboard.getInstance().startCameraStream(spindexer.LT, 0);
        telemetry.addData("team color", onRedTeam ? "RED" : "BLUE");
        telemetry.addLine();
        telemetry.addData("spindexer position", spindexer.getSpindexerAngle());
        telemetry.addData("spindexer target position", spindexer.getTargetAngle());
        telemetry.addData("spindexer power", spindexer.getSpindexerPower());
        telemetry.addData("spindexer mode", spindexer.inOuttakeMode() ? "outtake" : "intake");
        telemetry.addData("artifact previously detected", intakeCommand.artifactPreviouslyDetected);
        ArrayList<String> indexer = spindexer.getIndexerState();
        telemetry.addData("# artifacts", spindexer.getNumArtifacts());
        telemetry.addData("indexer state", indexer.get(0) + " " +
                indexer.get(1) + " " + indexer.get(2));
        telemetry.addData("spindexer" , spindexer.inOuttakeMode() ? "  " +
                indexer.get(2).charAt(0) : " " + indexer.get(2).charAt(0) + " " + indexer.get(1).charAt(0));
        telemetry.addData("state        ", spindexer.inOuttakeMode() ? " " + indexer.get(0).charAt(0)
                + " " + indexer.get(1).charAt(0) : "   " + indexer.get(0).charAt(0));
        telemetry.addLine();
        telemetry.addData("flywheel speed", outtake.getFlywheelSpeed()); // in rpm
        telemetry.addData("target speed", outtake.getTargetSpeed());
        telemetry.addData("flywheel power", outtake.flywheel.getPower());
        telemetry.addData("kicker position", outtake.getKickerPos());
        telemetry.addData("hood angle", outtake.getHoodAngle());
        telemetry.addData("turret position", outtake.getTurretPos());
        telemetry.addLine();
        telemetry.addData("tx", outtake.getTX());
        telemetry.addData("ta", outtake.getTA());
        telemetry.addData("apriltag ID", outtake.getApriltagID());

        telemetry.update();
    }
}