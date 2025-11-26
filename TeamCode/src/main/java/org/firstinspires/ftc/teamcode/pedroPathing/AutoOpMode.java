package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class AutoOpMode extends OpMode {
    private Follower follower;
    CloseAuto closeAuto;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        closeAuto = new CloseAuto(follower);
        follower.setStartingPose(closeAuto.startPose);
    }

    @Override
    public void loop() {
        follower.update();
        closeAuto.autoPathUpdate(follower);

        telemetry.addData("path state", closeAuto.getPathState());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }
}
