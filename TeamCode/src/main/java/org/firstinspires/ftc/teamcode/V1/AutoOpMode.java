package org.firstinspires.ftc.teamcode.V1;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
This is the main OpMode for our autonomous programs.
It creates and autonomous routine object (CloseAuto or FarAuto) which contains
information about the specific routine. You can also set the team color for
each routine. It also creates the follower, which is what makes the robot
actually follow the path.
*/

@Autonomous
public class AutoOpMode extends OpMode {
    // declare the follower
    private Follower follower;

    // declare an autonomous routine - CloseAuto or FarAuto
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
