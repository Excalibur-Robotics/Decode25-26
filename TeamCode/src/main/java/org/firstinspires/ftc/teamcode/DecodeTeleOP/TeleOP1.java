package org.firstinspires.ftc.teamcode.DecodeTeleOP;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DECODE.*;
import org.firstinspires.ftc.teamcode.V2.Subsystems.DrivetrainSubsystem;

//@TeleOp (name= "DECODE1")
public class TeleOP1 extends CommandOpMode {
    GamepadEx gp1;

    Out_take outTake;

    In_take intake;

    DrivetrainSubsystem drivetrain;

    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        // initialize hardware
        gp1 = new GamepadEx(gamepad1);
        outTake = new Out_take(hardwareMap);
        intake = new In_take(hardwareMap);
        drivetrain = new DrivetrainSubsystem(hardwareMap, gamepad1);
    }

    public void run(){
        // scheduler runs periodic drivetrain method for driving
        CommandScheduler.getInstance().run();

        // turn on outtake when right trigger is pressed
        if (gamepad1.right_trigger > 0.5){
            outTake.basicMovement(1);
        }
        else {
            outTake.basicMovement(0);
        }

        // turn on intake when left trigger is pressed
        if (gamepad1.left_trigger > 0.5){
            intake.movement(1);
        }
        else {
            intake.movement(0);
        }

        // reverse intake when left bumper is pressed
        if(gamepad1.left_bumper) {
            intake.movement(-1);
        }
        else {
            intake.movement(0);
        }
    }
}
