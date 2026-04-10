package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotFunctions {

    private DcMotor leftShooter;
    private DcMotor rightShooter;
    private DcMotor intake;
    private DcMotor midWheel;
    private Servo   trigger;

    // Constructor — maps both motors from hardware config
    public RobotFunctions(HardwareMap hardwareMap) {
        leftShooter  = hardwareMap.get(DcMotor.class, "LShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "RShooter");
        intake       = hardwareMap.get(DcMotor.class, "Intake");
        midWheel     = hardwareMap.get(DcMotor.class, "Belt");
        trigger      = hardwareMap.get(Servo.class, "Trigger");

        // Coast to a stop when power is cut
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        midWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // *** THIS IS THE KEY PART ***
        // One motor is reversed so both wheels push
        // the ball in the SAME direction through the gap.
        // If the ball isn't launching straight, try swapping which one is reversed.
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        midWheel.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // Spin both flywheels at the same power (0.0 to 1.0)
    public void spinShooters(double power) {
        leftShooter.setPower(power);
        rightShooter.setPower(power);
    }

    // Spin each flywheel at independent powers
    // Useful if one side is weaker or you want to tune spin on the ball
    public void spinIndependent(double leftPower, double rightPower) {
        leftShooter.setPower(leftPower);
        rightShooter.setPower(rightPower);
    }

    // Stop both flywheels
    public void stopShooters() {
        leftShooter.setPower(0.0);
        rightShooter.setPower(0.0);
    }

    // Check if flywheels are running
    public boolean isRunning() {
        return leftShooter.getPower() > 0 || rightShooter.getPower() > 0;
    }
    public void spinIntake(double power) {
        intake.setPower(power);
    }
    public void stopIntake() {
        intake.setPower(0.0);
    }
    public void shoot() throws InterruptedException {
        trigger.setPosition(0.0);
        wait(1000);
        trigger.setPosition(0.65);
        wait(500);
        spinIntake(0.8);
        midWheel.setPower(0.8);
        wait(1000);
        stopIntake();
        midWheel.setPower(0.0);
        trigger.setPosition(0.0);
        wait(1000);
        trigger.setPosition(0.65);
    }
}