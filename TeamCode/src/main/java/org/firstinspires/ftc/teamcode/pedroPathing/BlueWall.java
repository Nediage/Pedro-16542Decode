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
public class BlueWall extends OpMode {
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
        follower.setStartingPose(new Pose(59, 8, Math.toRadians(90)));

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
                                    new Pose(59, 8),
                                    new Pose(52, 90.4)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(315))
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
                                    new Pose(52, 90.4),
                                    new Pose(38.973, 94.781),
                                    new Pose(33.636, 82.5)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                    .addParametricCallback(0.1, robotFunctions::stopShooters)
                    .addParametricCallback(1.0, () -> robotFunctions.spinIntake(0.8))
                    .addPath(
                            new BezierLine(
                                    new Pose(33.636, 82.5),
                                    new Pose(24.0, 82.5)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addParametricCallback(1.0, robotFunctions::stopIntake)
                    .addPath(
                            new BezierLine(
                                    new Pose(24.0, 82.5),
                                    new Pose(52, 90.4)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
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
                                    new Pose(52, 90.4),
                                    new Pose(30.119, 70.998),
                                    new Pose(33.636, 58.5)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(180))
                    .addParametricCallback(0.1, robotFunctions::stopShooters)
                    .addParametricCallback(1.0, () -> robotFunctions.spinIntake(0.8))
                    .addPath(
                            new BezierLine(
                                    new Pose(33.473, 58.500),
                                    new Pose(24.000, 58.500)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addParametricCallback(1.0, robotFunctions::stopIntake)
                    .addPath(
                            new BezierLine(
                                    new Pose(24.000, 58.500),
                                    new Pose(52, 90.5)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    .addParametricCallback(0.5, () -> robotFunctions.spinShooters(1.0))
                    .addParametricCallback(1.0, () -> {
                        try {
                            robotFunctions.shoot();
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    })
                    .addPath(
                            new BezierLine(
                                    new Pose(52, 90.500),
                                    new Pose(58.5, 54)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(270))
                    .addParametricCallback(0.1, robotFunctions::stopShooters)
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