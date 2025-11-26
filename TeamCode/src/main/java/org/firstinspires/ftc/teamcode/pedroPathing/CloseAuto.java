package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class CloseAuto {
    public Pose startPose;
    public Pose shootPose;
    public Pose beforePickup;
    public Pose afterPickup;

    public PathChain goToFirstShoot;
    public PathChain goToPickup;
    public PathChain intakeBalls;
    public PathChain goToSecondShoot;

    private int pathState;

    public CloseAuto(Follower follower) {
        startPose = new Pose(122.0, 125.0, Math.toRadians(36));
        shootPose = new Pose(100.0, 107.0);
        beforePickup = new Pose(100.0, 84.0);
        afterPickup = new Pose(126.0, 84.0);

        goToFirstShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(44))
                .build();
        goToPickup = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, beforePickup))
                .setLinearHeadingInterpolation(Math.toRadians(44), 0)
                .build();
        intakeBalls = follower.pathBuilder()
                .addPath(new BezierLine(beforePickup, afterPickup))
                .setConstantHeadingInterpolation(0)
                .build();
        goToSecondShoot = follower.pathBuilder()
                .addPath(new BezierLine(afterPickup, shootPose))
                .setConstantHeadingInterpolation(0)
                .build();

        setPathState(0);
    }

    public void setPathState(int state) {
        pathState = state;
    }

    public int getPathState() {
        return pathState;
    }

    public void autoPathUpdate(Follower follower) {
        switch (pathState) {
            case 0:
                follower.followPath(goToFirstShoot);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(goToPickup);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(intakeBalls);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(goToSecondShoot);
                    setPathState(-1);
                }
                break;
        }
    }
}
