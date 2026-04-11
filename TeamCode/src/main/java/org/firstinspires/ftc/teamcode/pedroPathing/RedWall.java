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

@Autonomous(name = "RedWall", group = "Autonomous")
@Configurable
public class RedWall extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
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
        SHOOT_LOAD3,
        PARK
    }
    PathState pathState = PathState.DRIVE_TO_SHOOT;

    private PathChain driveToShoot1, driveToLoad1, intake1,
            driveToShoot2, driveToLoad2, intake2,
            driveToShoot3, park;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(82.5, 8, Math.toRadians(90)));
        follower.setMaxPower(0.6);

        robotFunctions = new RobotFunctions(hardwareMap);
        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    private void buildPaths() {
        driveToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(82.5, 8),
                        new Pose(86.5, 90.5)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(225))
                .addParametricCallback(0.5, () -> robotFunctions.spinShooters(1.0))
                .build();

        driveToLoad1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(86.5, 90.5),
                        new Pose(94.768, 91.185),
                        new Pose(110.364, 82.500)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .addParametricCallback(0.1, robotFunctions::stopShooters)
                .addParametricCallback(1.0, () -> robotFunctions.spinIntake(0.8))
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(110.364, 82.500),
                        new Pose(118, 82.500)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(1.0, robotFunctions::stopIntake)
                .build();

        driveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(118.000, 82.500),
                        new Pose(86.5, 90.5)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .addParametricCallback(0.5, () -> robotFunctions.spinShooters(1.0))
                .build();

        driveToLoad2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(87.041, 97.779),
                        new Pose(79.131, 67.904),
                        new Pose(110.364, 58.500)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .addParametricCallback(0.1, robotFunctions::stopShooters)
                .addParametricCallback(1.0, () -> robotFunctions.spinIntake(0.8))
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(110.364, 58.500),
                        new Pose(120.000, 58.500)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(1.0, robotFunctions::stopIntake)
                .build();

        driveToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(120.000, 58.500),
                        new Pose(86.5, 90.5)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(220))
                .addParametricCallback(0.5, () -> robotFunctions.spinShooters(1.0))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(86.500, 90.500),
                        new Pose(82, 54)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(90))
                .addParametricCallback(0.1, robotFunctions::stopShooters)
                .build();
    }

    @Override
    public void loop() {
        follower.update();
        robotFunctions.updateShoot();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shoot Done", robotFunctions.isShootDone());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case DRIVE_TO_SHOOT:
                follower.followPath(driveToShoot1, true);
                pathState = PathState.SHOOT_LOAD;
                break;
            case SHOOT_LOAD:
                if (!follower.isBusy()) {
                    robotFunctions.startShoot();
                    pathState = PathState.DRIVE_TO_LOAD;
                }
                break;
            case DRIVE_TO_LOAD:
                if (robotFunctions.isShootDone()) {
                    follower.followPath(driveToLoad1, true);
                    pathState = PathState.INTAKE;
                }
                break;
            case INTAKE:
                if (!follower.isBusy()) {
                    follower.followPath(intake1, true);
                    pathState = PathState.DRIVE_TO_SHOOT2;
                }
                break;
            case DRIVE_TO_SHOOT2:
                if (!follower.isBusy()) {
                    follower.followPath(driveToShoot2, true);
                    pathState = PathState.SHOOT_LOAD2;
                }
                break;
            case SHOOT_LOAD2:
                if (!follower.isBusy()) {
                    robotFunctions.startShoot();
                    pathState = PathState.DRIVE_TO_LOAD2;
                }
                break;
            case DRIVE_TO_LOAD2:
                if (robotFunctions.isShootDone()) {
                    follower.followPath(driveToLoad2, true);
                    pathState = PathState.INTAKE2;
                }
                break;
            case INTAKE2:
                if (!follower.isBusy()) {
                    follower.followPath(intake2, true);
                    pathState = PathState.DRIVE_TO_SHOOT3;
                }
                break;
            case DRIVE_TO_SHOOT3:
                if (!follower.isBusy()) {
                    follower.followPath(driveToShoot3, true);
                    pathState = PathState.SHOOT_LOAD3;
                }
                break;
            case SHOOT_LOAD3:
                if (!follower.isBusy()) {
                    robotFunctions.startShoot();
                    pathState = PathState.PARK;
                }
                break;
            case PARK:
                if (robotFunctions.isShootDone()) {
                    follower.followPath(park, true);
                }
                break;
        }
    }
}