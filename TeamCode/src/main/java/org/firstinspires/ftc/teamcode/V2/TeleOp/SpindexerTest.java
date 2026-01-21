package org.firstinspires.ftc.teamcode.V2.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.V2.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

/*
Simple TeleOp for V2 with the goal of just being able to launch artifacts.
Driver manually controls each individual movement, no automatic actions.
 */

@TeleOp
public class SpindexerTest extends CommandOpMode {
    SpindexerSubsystem spindexer;

    @Override
    public void initialize() {
        spindexer = new SpindexerSubsystem(hardwareMap);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

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


        //telemetry.addData("kicker position", outtake.getKickerPos());
        //telemetry.addData("flywheel speed", outtake.getFlywheelSpeed());
        telemetry.addData ("spindexer position", spindexer.getSpindexerAngle());
        /*ArrayList<String> indexer = spindexer.getIndexerState();
        telemetry.addData("indexer state", indexer.get(0) + " " +
                          indexer.get(1) + " " + indexer.get(2));
        */
        telemetry.update();
    }
}
