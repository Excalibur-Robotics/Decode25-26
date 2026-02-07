package org.firstinspires.ftc.teamcode.V2.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.V2.Commands.ActivateFlywheel;
import org.firstinspires.ftc.teamcode.V2.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootArtifact;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootColor;
import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

import java.util.ArrayList;

public class AutoRoutine {
    private int pathState;
    private boolean closeAuto;

    ArrayList<Pose> poses;
    ArrayList<PathChain> paths;

    public IntakeSubsystem intake;
    public SpindexerSubsystem spindexer;
    public OuttakeSubsystem outtake;

    public ActivateFlywheel activateFlywheel;
    public IntakeCommand activateIntake;
    public ShootColor shootPurple;
    public ShootColor shootGreen;
    public ShootArtifact shootArtifact;

    AutoRoutine(Follower follower, IntakeSubsystem intake, SpindexerSubsystem spindexer,
                OuttakeSubsystem outtake, boolean redTeam, boolean startClose) {
        if(startClose) {
            poses.add(new Pose(122, 125, Math.toRadians(36))); // start pose
            poses.add(new Pose(100.0, 107.0, Math.toRadians(44))); // shoot pose
            poses.add(new Pose(100.0, 84.0, 0)); // before pickup
            poses.add(new Pose(126.0, 84.0, 0)); // after pickup

            if(!redTeam) {
                poses.replaceAll(item -> new Pose(144 - item.getX(), item.getY(),
                        Math.PI-item.getHeading()));
            }

            // set paths, same for both colors
            paths.add(follower.pathBuilder() // go to first shoot
                    .addPath(new BezierLine(poses.get(0), poses.get(1)))
                    .setLinearHeadingInterpolation(poses.get(0).getHeading(), poses.get(1).getHeading())
                    .build());
            paths.add(follower.pathBuilder() // go to pick up
                    .addPath(new BezierLine(poses.get(1), poses.get(2)))
                    .setLinearHeadingInterpolation(poses.get(1).getHeading(), poses.get(2).getHeading())
                    .build());
            paths.add(follower.pathBuilder() // intake balls
                    .addPath(new BezierLine(poses.get(2), poses.get(3)))
                    .setConstantHeadingInterpolation(poses.get(2).getHeading())
                    .build());
            paths.add(follower.pathBuilder() // go to second shoot
                    .addPath(new BezierLine(poses.get(3), poses.get(1)))
                    .setConstantHeadingInterpolation(poses.get(2).getHeading())
                    .build());
        }
        else {
            poses.add(new Pose(87, 9, Math.PI/2)); // start pose
            poses.add(new Pose(100.0, 36, 0)); // before pickup
            poses.add(new Pose(126, 36, 0)); // after pickup
            poses.add(new Pose(87, 12, Math.PI/2)); // shoot pose

            if(!redTeam) {
                poses.replaceAll(item -> new Pose(144 - item.getX(), item.getY(),
                        Math.PI-item.getHeading()));
            }

            // set paths, same for both colors
            paths.add(follower.pathBuilder() // go to pick up
                    .addPath(new BezierLine(poses.get(0), poses.get(1)))
                    .setLinearHeadingInterpolation(poses.get(0).getHeading(), poses.get(1).getHeading())
                    .build());
            paths.add(follower.pathBuilder() // intake balls
                    .addPath(new BezierLine(poses.get(1), poses.get(2)))
                    .setConstantHeadingInterpolation(poses.get(1).getHeading())
                    .build());
            paths.add(follower.pathBuilder() // go to second shoot
                    .addPath(new BezierLine(poses.get(2), poses.get(3)))
                    .setLinearHeadingInterpolation(poses.get(2).getHeading(), poses.get(3).getHeading())
                    .build());
        }

        pathState = 0;
        closeAuto = startClose;

        this.intake = intake;
        this.spindexer = spindexer;
        this.outtake = outtake;

        activateFlywheel = new ActivateFlywheel(outtake, spindexer);
        activateIntake = new IntakeCommand(intake, spindexer);
        shootPurple = new ShootColor(outtake, spindexer, "purple");
        shootGreen = new ShootColor(outtake, spindexer, "green");
        shootArtifact = new ShootArtifact(outtake, spindexer);
    }

    // set the path state
    public void setPathState(int state) {
        pathState = state;
    }
    // get the current path state for debugging
    public int getPathState() {
        return pathState;
    }
    public ArrayList<Pose> getPoses() {
        return poses;
    }
    public ArrayList<PathChain> getPaths() {
        return paths;
    }

    public void autoPathUpdate(Follower follower) {
        if(closeAuto) {
            switch (pathState) {
                case 0:
                    follower.followPath(paths.get(0));
                    activateFlywheel.schedule();
                    setPathState(1);
                    break;
                case 1:
                    if (!follower.isBusy()) {
                        if (spindexer.getNumArtifacts() > 0) {
                            shootArtifact.schedule();
                        } else {
                            follower.followPath(paths.get(1));
                            setPathState(2);
                        }
                    }
                    break;
                case 2:
                    if (!follower.isBusy()) {
                        activateIntake.schedule();
                        follower.followPath(paths.get(2));
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (!follower.isBusy()) {
                        activateFlywheel.schedule();
                        follower.followPath(paths.get(3));
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (!follower.isBusy()) {
                        if (spindexer.getNumArtifacts() > 0) {
                            shootArtifact.schedule();
                        } else {
                            setPathState(-1);
                        }
                    }
                    break;
            }
        }
        else {
            switch (pathState) {
                case 0:
                    activateFlywheel.schedule();
                    if(spindexer.getNumArtifacts() > 0) {
                        shootArtifact.schedule();
                    }
                    else {
                        follower.followPath(paths.get(0));
                        setPathState(1);
                    }
                    break;
                case 1:
                    if(!follower.isBusy()) {
                        activateIntake.schedule();
                        follower.followPath(paths.get(1));
                        setPathState(2);
                    }
                    break;
                case 2:
                    if(!follower.isBusy()) {
                        activateFlywheel.schedule();
                        follower.followPath(paths.get(2));
                        setPathState(3);
                    }
                    break;
                case 3:
                    if(!follower.isBusy()) {
                        if(spindexer.getNumArtifacts() > 0) {
                            shootArtifact.schedule();
                        }
                        else {
                            setPathState(-1);
                        };
                    }
                    break;
            }
        }
    }
}
