package org.firstinspires.ftc.teamcode.DECODE;

import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.V1.Subsystems.DrivetrainSubsystem;

import java.util.List;

@Configurable
@TeleOp(name="LimelightTest")
public class LimelightTest extends OpMode {
    private Limelight3A limelight;
    DrivetrainSubsystem drivetrain;
    public DcMotor turret;
    GamepadEx gamepad = new GamepadEx(gamepad1);
    PDController turretController;
    public static double kP = 0.05; // need to tune
    public static double kD = 0.0; // need to tune

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        drivetrain = new DrivetrainSubsystem(hardwareMap, gamepad1);
        turret = hardwareMap.get(DcMotor.class, "turret");
        turretController = new PDController(kP, kD);
        turretController.setSetPoint(0);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        // get result from limelight
        LLResult result = limelight.getLatestResult();
        int id = 0;
        double tx = 0;
        double ty = 0;
        double ta = 0;
        String poseString = "";
        if(result != null && result.isValid()) {
            // get AprilTag ID
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                id = fiducials.get(0).getFiducialId();
            }
            // position of target
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();
            // Pose of robot
            Pose3D botPose = result.getBotpose();
            poseString = botPose.toString();
        }

        /* This was for making the whole robot turn toward the apriltag
           instead of a turret
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double yaw = 0;
        if(gamepad1.right_stick_x > 0.05) {
            yaw = gamepad1.right_stick_x;
        }
        else {
            yaw = -turretController.calculate(tx);
        }
        drivetrain.moveRobot(x, y, yaw);
        */

        // turn turret toward apriltag
        double turretPower = turretController.calculate(tx);
        turret.setPower(turretPower);

        // Telemetry
        // check if an AprilTag is in view
        if(id == 0) {
            telemetry.addData("AprilTag ID", "No AprilTag in view");
            telemetry.addData("Motif/Goal", "");
        }
        // display motif/goal of the AprilTag
        else {
            telemetry.addData("AprilTag ID", id);
            if (id == 21)
                telemetry.addData("Motif/Goal", "green, purple, purple");
            else if (id == 22)
                telemetry.addData("Motif/Goal", "purple, green, purple");
            else if (id == 23)
                telemetry.addData("Motif/Goal", "purple, purple, green");
            else if (id == 20)
                telemetry.addData("Motif/Goal", "Blue Goal");
            else if (id == 24)
                telemetry.addData("Motif/Goal", "Red Goal");
            /*
            Each Id has its own ID. It is critical for limelight to understand what
            its seeing so that is able to interpret what the color code is for the
            current robot.
             */
        }
        // display target position and bot pose
        telemetry.addLine();
        telemetry.addData("tx", tx);
        telemetry.addData("ty", ty);
        telemetry.addData("ta", ta);
        telemetry.addData("Pose", poseString);
    }
    /*
    This adds all the data to the driver hub so
    that the driver knows what they are seeing
     */
}