package org.firstinspires.ftc.teamcode.DecodeTeleOP;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DECODE.*;

@TeleOp (name= "DECODE1")
public class TeleOP1 extends CommandOpMode {
    GamepadEx gp1;

    Out_take outTake;

    In_take intake;

    DrivetrainSubsystem drivetrain;

    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();
        gp1 = new GamepadEx(gamepad1);
        outTake = new Out_take(hardwareMap);
        intake = new In_take(hardwareMap);
        drivetrain = new DrivetrainSubsystem(hardwareMap, gamepad1);
    }

    public void run(){
        CommandScheduler.getInstance().run();
        if (gamepad1.right_trigger > 0.5){
            outTake.basicMovement(1);
        }
        else {
            outTake.basicMovement(0);
        }

        if (gamepad1.left_trigger > 0.5){
            intake.movement(-1);
        }
        else {
            intake.movement(0);
        }
        if(gamepad1.left_bumper) {
            intake.movement(1);
        }
        else {
            intake.movement(0);
        }
    }
}
