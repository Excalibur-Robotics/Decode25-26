package org.firstinspires.ftc.teamcode.DECODE;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.V1.Subsystems.DrivetrainSubsystem;

@Autonomous
public class BasicAuto extends LinearOpMode {
    public DrivetrainSubsystem drivetrain;
    public In_take intake;
    public Out_take outtake;
    ElapsedTime timer = new ElapsedTime();
    GamepadEx gamepad = new GamepadEx(gamepad1);

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize hardware
        drivetrain = new DrivetrainSubsystem(hardwareMap, gamepad1);
        intake = new In_take(hardwareMap);
        outtake = new Out_take(hardwareMap);

        waitForStart();

        timer.reset();
        while(timer.seconds() < 0.5) {
            drivetrain.moveRobot(0, -0.5, 0);
        }
        drivetrain.moveRobot(0,0,0);

        // turn on outtake and spin up for 0.5 seconds
        timer.reset();
        while(timer.seconds() < 0.5) {
            outtake.basicMovement(1);
        }

        // turn on intake for 3 seconds
        timer.reset();
        while(timer.seconds() < 3) {
            intake.movement(0.5);
        }
        intake.movement(0);
        outtake.basicMovement(0);

        // move robot back and right
        timer.reset();
        while(timer.seconds() < 1) {
            drivetrain.moveRobot(0, -0.5, 0);
        }
        drivetrain.moveRobot(0,0,0);
        timer.reset();
        while(timer.seconds() < 1.5) {
            drivetrain.moveRobot(0.5, 0, 0);
        }
        drivetrain.moveRobot(0,0,0);
    }
}
