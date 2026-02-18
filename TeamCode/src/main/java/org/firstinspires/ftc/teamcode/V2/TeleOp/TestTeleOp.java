package org.firstinspires.ftc.teamcode.V2.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.V2.Subsystems.EndgameSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

@TeleOp
public class TestTeleOp extends CommandOpMode {
    EndgameSubsystem endgame;

    @Override
    public void initialize() {
        endgame = new EndgameSubsystem(hardwareMap);
    }

    @Override
    public void run() {
        if(gamepad1.dpad_up) {
            endgame.activateEndgame();
        }
        if(gamepad1.dpad_down) {
            endgame.resetServos();
        }

        telemetry.addData("endgame activated", endgame.getServoPos() > 0);
        telemetry.update();
    }
}
