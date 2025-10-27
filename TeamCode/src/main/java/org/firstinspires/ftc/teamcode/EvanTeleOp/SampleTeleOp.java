package org.firstinspires.ftc.teamcode.EvanTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="SampleTeleOp")
public class SampleTeleOp extends OpMode {
    Drivetrain drivetrain = new Drivetrain();
    public Intake intake = new Intake();
    public Outtake outtake = new Outtake();

    boolean previousX;

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        intake.init(hardwareMap);
        outtake.init(hardwareMap);

        previousX = false;
    }

    @Override
    public void loop() {
        // input stick positions and set drive motor powers
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double yaw = gamepad1.right_stick_x;
        drivetrain.moveRobot(x, y, yaw);

        // launch an artifact
        if(gamepad1.x || outtake.getState()) {
            outtake.shoot();
        }

        // toggle intake on and off
        if(gamepad1.a && !previousX) {
            intake.toggleIntake();
        }
        previousX = gamepad1.x;
    }
}
