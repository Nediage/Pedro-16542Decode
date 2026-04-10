package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.RobotFunctions;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "BlueDepot", group = "Autonomous")
@Configurable // Panels
public class BlueDepot extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private RobotFunctions robotFunctions;
    public enum PathState {
        DRIVE_TO_SHOOT,
        SHOOT_PRELOAD,
        DRIVE_TO_LOAD,
        INTAKE,
        DRIVE_TO_SHOOT2
    }
    PathState pathState;

    private final Pose startPose = new Pose(56.95934466019417, 97.7791262135922, Math.toRadians(324));
    private final Pose shootPose = new Pose(56.959, 97.779, Math.toRadians(320));


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.95934466019417, 97.7791262135922, Math.toRadians(324)));

        // Paths defined in the Paths class
        Paths paths = new Paths(follower, robotFunctions); // Build paths
        robotFunctions = new RobotFunctions(hardwareMap);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

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
                                    new Pose(22.342, 123.055),
                                    new Pose(56.959, 97.779)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(324), Math.toRadians(320))
                    .addParametricCallback(1.0, () -> {
                        try {
                            robotFunctions.shoot();
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    })
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.959, 97.779),
                                    new Pose(56.884, 80.940),
                                    new Pose(33.636, 82.889)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(180))
                    .addParametricCallback(1.0, () -> robotFunctions.spinIntake(0.8))
                    .addPath(
                            new BezierLine(
                                    new Pose(33.636, 82.889),
                                    new Pose(23.539, 82.687)
                            )
                    )
                    .addParametricCallback(1.0, robotFunctions::stopIntake)
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case DRIVE_TO_SHOOT:
                follower.followPath();
        }
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return 0;
    }
}