package org.firstinspires.ftc.teamcode.V2.TeleOp;

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

        // set buttons/triggers
        leftTrigger = new Trigger(() -> gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        rightTrigger = new Trigger(() -> gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        rightBumper = new GamepadButton(gp2, GamepadKeys.Button.RIGHT_BUMPER);
        X = new GamepadButton(gp2, GamepadKeys.Button.X);
        B = new GamepadButton(gp2, GamepadKeys.Button.B);
        A = new GamepadButton(gp2, GamepadKeys.Button.A);

        // Bind buttons/triggers with commands
        leftTrigger.whileActiveOnce(new IntakeCommand(intake, spindexer));
        rightTrigger.whenActive(new ActivateFlywheel(outtake, spindexer, gamepad1));
        rightBumper.whenPressed(new ShootArtifact(outtake, spindexer));
        X.whenPressed(new ShootColor(outtake, spindexer, "green"));
        B.whenPressed(new ShootColor(outtake, spindexer, "purple"));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        drivetrain.teleOpDrive(gamepad1);
        outtake.calculateTurret(outtake.getTX());

        telemetry.addData ("spindexer position", spindexer.getSpindexerAngle());
        ArrayList<String> indexer = spindexer.getIndexerState();
        telemetry.addData("indexer state", indexer.get(0) + " " + indexer.get(1) + " " + indexer.get(2));
        telemetry.update();
    }
}