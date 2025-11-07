package org.firstinspires.ftc.teamcode.DECODE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BasicAuto extends LinearOpMode {
    public DrivetrainSubsystem drivetrain;
    public In_take intake;
    public Out_take outtake;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new DrivetrainSubsystem(hardwareMap);
        intake = new In_take(hardwareMap);
        outtake = new Out_take(hardwareMap);

        waitForStart();

        timer.reset();
        while(timer.seconds() < 1.5) {
            drivetrain.moveRobot(0, -0.5, 0);
        }
        drivetrain.moveRobot(0,0,0);
        timer.reset();
        while(timer.seconds() < 0.5) {
            outtake.basicMovement(1);
        }
        timer.reset();
        while(timer.seconds() < 3) {
            intake.movement(0.25);
        }
        intake.movement(0);
        outtake.basicMovement(0);
        timer.reset();
        while(timer.seconds() < 1.5) {
            drivetrain.moveRobot(0.5, 0, 0);
        }
        drivetrain.moveRobot(0,0,0);
    }
}
