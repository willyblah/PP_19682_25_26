package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.panelConstants.*;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.*;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Shooter {
    public DcMotorEx leftShooter;
    public DcMotorEx rightShooter;
    public DcMotorEx turret;
    public Servo panel;

    public static double targetVelocity = 0, spKp = 0.027, spKi = 0.0, spKd = 0.236, basePower = 0.17, tor = 0.025, hoodCorrection = 0.045;
    ElapsedTime dt = new ElapsedTime();
    double error, lastError, integral, derivative, lastTarget = 0.0, power = 0.0;

    public void init(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, LEFT_SHOOTER);
        rightShooter = hardwareMap.get(DcMotorEx.class, RIGHT_SHOOTER);
        turret = hardwareMap.get(DcMotorEx.class, TURRET);
        panel = hardwareMap.get(Servo.class, PANEL);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turret.setPositionPIDFCoefficients(15);

        leftShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        rightShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
    }

    public void reset() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setPower(0);
    }

    public void setShooterVelocity(double velocity) {
        leftShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        rightShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        leftShooter.setVelocity(velocity);
        rightShooter.setVelocity(velocity);
    }

    public double getShooterVelocity() {
        return (rightShooter.getVelocity() + leftShooter.getVelocity()) / 2.0;
    }

    public boolean shooterReady(double target) {
        return Math.abs(getShooterVelocity() - target) <= VELOCITY_TOR;
    }

    public boolean shooterReady() {
        return Math.abs(getShooterVelocity() - targetVelocity) <= VELOCITY_TOR;
    }

    public void shooterHold() {
        leftShooter.setVelocity(1360);
        rightShooter.setVelocity(1360);
    }

    public void shooterStop() {
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }

    public void turretRotateLeft() {
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setPower(0.5);
    }

    public void turretRotateRight() {
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setPower(-0.5);
    }

    public void turretRotateStop() {
        turret.setPower(0);
    }

    public void setShooter(double panel, double velocity) {
        panelTo(Range.clip(panel, 0.0, 1.0));
        setShooterVelocity(velocity);
    }

    public void setShooterByDis(double distance) {
        targetVelocity = f(-0.00039, 0.1345118, -5.930571, 1286.731, distance);
        leftShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        rightShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        setShooter(Range.clip(f(0.0, 0.000002082898, 0.003827418, 0.05630374, distance), 0.2, 0.66), targetVelocity);
    }

    public double calculateIntakePower() {
        if (targetVelocity > 1450) return 0.8;
        return 1.0;
    }

    public long calculateGap() {
        return (long) Math.max(0.0, f(0.0, -6.13e-5, 0.806, -1342.0, targetVelocity)) + 80;
    }

    public double f(double a, double b, double c, double d, double x) {
        return a * Math.pow(x, 3) + b * Math.pow(x, 2) + c * x + d;
    }

    public int getTurretPosition() {
        return turret.getCurrentPosition();
    }

    public double getTurretDegree() {
        return getTurretPosition() / TURRET_FULL_RANGE_ENCODER * TURRET_FULL_RANGE_DEGREE;
    }

    public void turretToDegree(double degree) {
        int position = (int) (degree * TURRET_FULL_RANGE_ENCODER / TURRET_FULL_RANGE_DEGREE);
        turret.setTargetPosition(position);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
    }

    public void panelTo(double pos) {
        panel.setPosition(pos);
    }
}
