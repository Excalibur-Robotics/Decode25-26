package org.firstinspires.ftc.teamcode.DECODE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name="LimelightTest")
public class LimelightTest extends OpMode {
    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
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

        // check if an AprilTag is in view
        if(id == 0) {
            telemetry.addData("AprilTag ID", "No AprilTag in view");
            telemetry.addData("Motif", "");
        }
        // display motif/goal of the AprilTag
        else {
            telemetry.addData("AprilTag ID", id);
            if (id == 21)
                telemetry.addData("Motif", "green, purple, purple");
            else if (id == 22)
                telemetry.addData("Motif", "purple, green, purple");
            else if (id == 23)
                telemetry.addData("Motif", "purple, purple, green");
            else if (id == 20)
                telemetry.addData("Motif", "Blue Goal");
            else if (id == 24)
                telemetry.addData("Motif", "Red Goal");
        }
        // display target position and bot pose
        telemetry.addLine();
        telemetry.addData("tx", tx);
        telemetry.addData("tx", ty);
        telemetry.addData("tx", ta);
        telemetry.addData("Pose", poseString);
    }
}