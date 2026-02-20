package org.firstinspires.ftc.teamcode.V2.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.V2.Commands.ActivateFlywheel;
import org.firstinspires.ftc.teamcode.V2.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootArtifact;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootColor;
import org.firstinspires.ftc.teamcode.V2.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

public class States_TeleOP extends CommandOpMode {
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    OuttakeSubsystem outtake;
    DrivetrainSubsystem drivetrain;
    GamepadEx gp1;

    Trigger leftTrigger;
    Trigger rightTrigger;
    Button rightBumper;
    Button X;
    Button B;
    Button A;
    Trigger spindexerRotating;

    public void initialize(){
        intake = new IntakeSubsystem(hardwareMap);
        spindexer= new SpindexerSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        drivetrain = new DrivetrainSubsystem(hardwareMap);

        gp1= new GamepadEx(gamepad1);

        // initialize triggers
        leftTrigger = new Trigger(() -> gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        rightTrigger = new Trigger(() -> gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        X = new GamepadButton(gp1, GamepadKeys.Button.X);
        B = new GamepadButton(gp1, GamepadKeys.Button.B);
        A = new GamepadButton(gp1, GamepadKeys.Button.A);
        spindexerRotating = new Trigger(() -> spindexer.getSpindexerPower() > 0.05);

        leftTrigger.whileActiveOnce(new IntakeCommand(intake, spindexer));
        rightTrigger.whenActive(new ActivateFlywheel(outtake, gamepad1));
        X.whenPressed(new ConditionalCommand(
                new ShootArtifact(outtake, spindexer),
                new InstantCommand(),
                () -> outtake.getFlywheelSpeed() > outtake.getTargetSpeed() - 30));
        A.whenPressed(new ConditionalCommand(
                new ShootColor(outtake, spindexer, "green"),
                new InstantCommand(),
                () -> outtake.getFlywheelSpeed() > outtake.getTargetSpeed() - 30));
    }


}
