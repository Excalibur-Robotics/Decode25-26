package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class AutoOpMode extends OpMode {
    // declare the follower, which is what makes the robot follow the path
    private Follower follower;

    // declare an autonomous routine, change for close or far: CloseAuto or FarAuto
    // this contains the paths and the state machine to determine which path to follow
    CloseAuto routine;

    @Override
    public void init() {
        // initialize follower
        follower = Constants.createFollower(hardwareMap);

        // initialize auto routine, set team color: red is true, blue is false
        routine = new CloseAuto(follower, true);

        // set the starting pose of the robot
        follower.setStartingPose(routine.startPose);
    }

    @Override
    public void loop() {
        // This is the main loop, which determines the current path
        // and makes the robot follow it
        follower.update();
        routine.autoPathUpdate(follower);

        // telemetry for debugging: current path and robot pose
        telemetry.addData("path state", routine.getPathState());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }
}
