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
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BlueDepot", group = "Autonomous")
@Configurable
public class BlueDepot extends OpMode {
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

    // Flat path chains — no nested class
    private PathChain driveToShoot1, driveToLoad1, intake1,
            driveToShoot2, driveToLoad2, intake2,
            driveToShoot3, park;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22.342, 123.055, Math.toRadians(324)));
        follower.setMaxPower(0.6);

        robotFunctions = new RobotFunctions(hardwareMap);
        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    private void buildPaths() {
        driveToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(22.342, 123.055),
                        new Pose(56.959, 97.779)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(320))
                .addParametricCallback(0.2, () -> robotFunctions.spinShooters(1.0))
                .build(); // startShoot() called in state machine after path ends

        driveToLoad1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.959, 97.779),
                        new Pose(56.884, 80.940),
                        new Pose(33.636, 82.889)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(180))
                .addParametricCallback(0.1, robotFunctions::stopShooters)
                .addParametricCallback(1.0, () -> robotFunctions.spinIntake(0.8))
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(33.636, 82.889),
                        new Pose(23.539, 82.687)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(1.0, robotFunctions::stopIntake)
                .build();

        driveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(23.539, 82.687),
                        new Pose(56.959, 97.779)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(320))
                .addParametricCallback(0.5, () -> robotFunctions.spinShooters(1.0))
                .build();

        driveToLoad2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.959, 97.779),
                        new Pose(30.119, 70.998),
                        new Pose(33.473, 58.500)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(180))
                .addParametricCallback(0.1, robotFunctions::stopShooters)
                .addParametricCallback(1.0, () -> robotFunctions.spinIntake(0.8))
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(33.473, 58.500),
                        new Pose(24.000, 58.500)
                ))
                .setTangentHeadingInterpolation()
                .addParametricCallback(1.0, robotFunctions::stopIntake)
                .build();

        driveToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(24.000, 58.500),
                        new Pose(56.959, 97.779)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(320))
                .addParametricCallback(0.5, () -> robotFunctions.spinShooters(1.0))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.959, 97.779),
                        new Pose(38.309, 79.727)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(225))
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
                // Path finished — now trigger shoot and wait for it
                if (!follower.isBusy()) {
                    robotFunctions.startShoot();
                    pathState = PathState.DRIVE_TO_LOAD;
                }
                break;
            case DRIVE_TO_LOAD:
                // Wait for shoot to finish, then drive
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
                    // done
                }
                break;
        }
    }
}