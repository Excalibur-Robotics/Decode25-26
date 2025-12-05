package org.firstinspires.ftc.teamcode.V1;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.V1.Commands.ActivateFlywheel;
import org.firstinspires.ftc.teamcode.V1.Commands.AimTurret;
import org.firstinspires.ftc.teamcode.V1.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.V1.Commands.ShootArtifact;
import org.firstinspires.ftc.teamcode.V1.Commands.ShootColor;
import org.firstinspires.ftc.teamcode.V1.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.V1.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V1.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V1.Subsystems.SpindexerSubsystem;

public class V1TeleOp extends CommandOpMode {
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    OuttakeSubsystem outtake;
    DrivetrainSubsystem drivetrain;

    GamepadEx gp1;
    GamepadEx gp2;

    Trigger leftTrigger;
    Trigger rightTrigger;
    Button rightBumper;
    Button X;
    Button B;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        gp1 = new GamepadEx(gamepad1); // driving
        gp2 = new GamepadEx(gamepad2); // intake/outtake

        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        drivetrain = new DrivetrainSubsystem(hardwareMap, gp1);

        leftTrigger = new Trigger(() -> gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        rightTrigger = new Trigger(() -> gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        rightBumper = new GamepadButton(gp2, GamepadKeys.Button.RIGHT_BUMPER);
        X = new GamepadButton(gp2, GamepadKeys.Button.X);
        B = new GamepadButton(gp2, GamepadKeys.Button.B);

        leftTrigger.whileActiveOnce(new IntakeCommand(intake, spindexer));
        rightTrigger.whileActiveOnce(new ActivateFlywheel(outtake, spindexer));
        rightBumper.whenPressed(new ShootArtifact(outtake, spindexer));
        X.whenPressed(new ShootColor(outtake, spindexer, "green"));
        B.whenPressed(new ShootColor(outtake, spindexer, "purple"));
        CommandScheduler.getInstance().setDefaultCommand(outtake, new AimTurret(outtake));
    }
}
