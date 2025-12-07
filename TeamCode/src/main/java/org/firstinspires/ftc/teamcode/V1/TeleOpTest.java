package org.firstinspires.ftc.teamcode.V1;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.V1.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.V1.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V1.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V1.Subsystems.SpindexerSubsystem;

import java.util.ArrayList;

/*
OpMode to test basic teleop functions for V1
Not actual teleop that would be used for matches
 */

public class TeleOpTest extends CommandOpMode {
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    OuttakeSubsystem outtake;
    DrivetrainSubsystem drivetrain;

    GamepadEx gamepadEx = new GamepadEx(gamepad1);

    @Override
    public void initialize() {
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        drivetrain = new DrivetrainSubsystem(hardwareMap, gamepadEx);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        if(gamepad1.y) {
            outtake.kickUp();
        }
        else {
            outtake.resetKicker();
        }

        if(gamepad1.left_trigger > 0.5) {
            intake.activateIntake();
        }
        else {
            intake.stopIntake();
        }

        if(gamepad1.right_trigger > 0.5) {
            outtake.setFlywheelPower(1);
        }
        else {
            outtake.setFlywheelPower(0);
        }

        if(gamepad1.dpadLeftWasPressed()) {
            spindexer.rotateCCW();
        }

        if(gamepad1.dpadRightWasPressed()) {
            spindexer.rotateCW();
        }

        telemetry.addData("kicker position", outtake.getKickerPos());
        telemetry.addData ("spindexer position", spindexer.getSpindexerAngle());
        ArrayList<String> indexer = spindexer.getIndexerState();
        telemetry.addData("indexer state", indexer.get(0) + " " +
                          indexer.get(1) + " " + indexer.get(2));
        telemetry.update();
    }
}
