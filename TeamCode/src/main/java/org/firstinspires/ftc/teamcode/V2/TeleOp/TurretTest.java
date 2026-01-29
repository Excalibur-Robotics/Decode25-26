package org.firstinspires.ftc.teamcode.V2.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.V2.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

/*
Simple TeleOp for V2 with the goal of just being able to launch artifacts.
Driver manually controls each individual movement, no automatic actions.
 */

@Config
//@TeleOp
public class TurretTest extends CommandOpMode {
    DcMotor turret;
    Servo hoodR;
    Servo hoodL;
    DcMotor flywheel;
    Limelight3A limelight;

    //GamepadEx gamepadEx = new GamepadEx(gamepad1);

    private PIDController turretController;
    // turret tracking PID constants
    public static double kP = 0.025;
    public static double kD = 0.00005;

    // hood servo positions
    public static double hoodUp = 0.5;
    public static double hoodDown = 0.0;

    @Override
    public void initialize() {
        turret = hardwareMap.get(DcMotor.class, "turret");
        hoodR = hardwareMap.get(Servo.class, "hoodR");
        hoodL = hardwareMap.get(Servo.class, "hoodL");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        turretController = new PIDController(kP, 0, kD);
        turretController.setSetPoint(0);
    }

    @Override
    public void run() {
        //CommandScheduler.getInstance().run();

        // flywheel - right trigger
        if(gamepad1.right_trigger > 0.5) {
            flywheel.setPower(1);
        }
        else {
            flywheel.setPower(0);
        }

        // move hood up - dpad up
        if(gamepad1.dpad_up) {
            hoodR.setPosition(-hoodUp);
            hoodL.setPosition(hoodUp);
        }

        // move hood down - dpad down
        if(gamepad1.dpad_down) {
            hoodR.setPosition(-hoodDown);
            hoodL.setPosition(hoodDown);
        }

        // aim turret with limelight
        LLResult llData = limelight.getLatestResult();
        double tx = 0;
        if (llData != null && llData.isValid()) {
            tx = llData.getTx();
        }
        double power = turretController.calculate(tx);
        turret.setPower(power);


        //telemetry.addData("kicker position", outtake.getKickerPos());
        //telemetry.addData("flywheel speed", outtake.getFlywheelSpeed());
        telemetry.addData("tx", tx);
        telemetry.update();
    }
}
