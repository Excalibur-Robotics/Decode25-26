package org.firstinspires.ftc.teamcode.V2.Autos;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.V2.Commands.ActivateFlywheel;
import org.firstinspires.ftc.teamcode.V2.Commands.AimTurret;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootArtifact;
import org.firstinspires.ftc.teamcode.V2.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

/*
Basic Auto without pedro pathing to score three preloads
 */
public class SimpleAuto extends OpMode {
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    OuttakeSubsystem outtake;
    DrivetrainSubsystem drivetrain;
    GamepadEx gp1 = new GamepadEx(gamepad1);
    ElapsedTime timer = new ElapsedTime();
    int autoState;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        drivetrain = new DrivetrainSubsystem(hardwareMap, gamepad1);
        autoState = 0;
        CommandScheduler.getInstance().setDefaultCommand(outtake, new AimTurret(outtake));
        CommandScheduler.getInstance().schedule(new ActivateFlywheel(outtake, spindexer, gamepad2));
    }

    @Override
    public void start() {
        drivetrain.moveRobot(0, -0.5, 0);
        timer.reset();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        if(timer.milliseconds() > 2000) {
            drivetrain.moveRobot(0,0,0);
            CommandScheduler.getInstance().schedule(new ShootArtifact(outtake, spindexer));
        }

    }
}
