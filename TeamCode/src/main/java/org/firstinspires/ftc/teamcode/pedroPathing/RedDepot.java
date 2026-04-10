package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "BlueDepot", group = "Autonomous")
@Configurable // Panels
public class RedDepot extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private RobotFunctions robotFunctions;
    public enum PathState {
        DRIVE_TO_SHOOT,
        SHOOT_LOAD,
        DRIVE_TO_LOAD,
        INTAKE,
        DRIVE_TO_SHOOT2,
        SHOOT_LOAD2,
        DRIVE_TO_LOAD2,
        INTAKE2,
        DRIVE_TO_SHOOT3,
        SHOOT_LOAD3
    }
    PathState pathState = PathState.DRIVE_TO_SHOOT;

    private Paths paths;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22.342, 123.055, Math.toRadians(324)));

        // Paths defined in the Paths class
        robotFunctions = new RobotFunctions(hardwareMap);
        paths = new Paths(follower, robotFunctions); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain MainChain;

        public Paths(Follower follower, RobotFunctions robotFunctions) {
            MainChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(120.500, 123.055),
                                    new Pose(87.041, 97.400)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(220))
                    .addParametricCallback(0.5, () -> robotFunctions.spinShooters(1.0))
                    .addParametricCallback(1.0, () -> {
                        try {
                            robotFunctions.shoot();
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    })
                    .addPath(
                            new BezierCurve(
                                    new Pose(87.041, 97.400),
                                    new Pose(82.774, 90.221),
                                    new Pose(110.364, 82.500)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(0))
                    .addParametricCallback(0.1, robotFunctions::stopShooters)
                    .addParametricCallback(1.0, () -> robotFunctions.spinIntake(0.8))
                    .addPath(
                            new BezierLine(
                                    new Pose(110.364, 82.500),
                                    new Pose(120.000, 82.500)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addParametricCallback(1.0, robotFunctions::stopIntake)
                    .addPath(
                            new BezierLine(
                                    new Pose(120.000, 82.500),
                                    new Pose(87.041, 97.779)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(220))
                    .addParametricCallback(0.5, () -> robotFunctions.spinShooters(1.0))
                    .addParametricCallback(1.0, () -> {
                        try {
                            robotFunctions.shoot();
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    })
                    .addPath(
                            new BezierCurve(
                                    new Pose(87.041, 97.779),
                                    new Pose(79.131, 67.904),
                                    new Pose(110.364, 58.500)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(0))
                    .addParametricCallback(0.1, robotFunctions::stopShooters)
                    .addParametricCallback(1.0, () -> robotFunctions.spinIntake(0.8))
                    .addPath(
                            new BezierLine(
                                    new Pose(110.364, 58.500),
                                    new Pose(120.000, 58.500)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addParametricCallback(1.0, robotFunctions::stopIntake)
                    .addPath(
                            new BezierLine(
                                    new Pose(120.000, 58.500),
                                    new Pose(87.041, 97.779)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(220))
                    .addParametricCallback(0.5, () -> robotFunctions.spinShooters(1.0))
                    .addParametricCallback(1.0, () -> {
                        try {
                            robotFunctions.shoot();
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    })
                    .build();
        }
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case DRIVE_TO_SHOOT:
                follower.followPath(paths.MainChain, true);
                pathState = PathState.SHOOT_LOAD;
                break;
            case SHOOT_LOAD:
                if (!follower.isBusy()) {
                    pathState = PathState.DRIVE_TO_LOAD; // or whatever is next
                }
                break;
            case DRIVE_TO_LOAD:
                if (!follower.isBusy()) {
                    pathState = PathState.INTAKE;
                }
                break;
            case INTAKE:
                if (!follower.isBusy()) {
                    pathState = PathState.DRIVE_TO_SHOOT2;
                }
                break;
            case DRIVE_TO_SHOOT2:
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_LOAD2;
                }
                break;
            case SHOOT_LOAD2:
                if (!follower.isBusy()) {
                    pathState = PathState.DRIVE_TO_LOAD2;
                }
                break;
            case DRIVE_TO_LOAD2:
                if (!follower.isBusy()) {
                    pathState = PathState.INTAKE2;
                }
                break;
            case INTAKE2:
                if (!follower.isBusy()) {
                    pathState = PathState.DRIVE_TO_SHOOT3;
                }
                break;
            case DRIVE_TO_SHOOT3:
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_LOAD3;
                }
        }
    }
}