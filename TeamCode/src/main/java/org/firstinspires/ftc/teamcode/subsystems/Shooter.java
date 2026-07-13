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

    public static double targetVelocity = 0,  hoodCorrection = 0.045;
    public static double targetPanel = 0;
    public static double RPM_A = -0.0004036439;
    public static double RPM_B =  0.1057019636;
    public static double RPM_C = -1.8999025678;
    public static double RPM_D =  1153.0825835107;

    public static double PANEL_A =  2.39680e-7;
    public static double PANEL_B = -9.05038e-5;
    public static double PANEL_C =  0.015501136306;
    public static double PANEL_D = -0.302168750944;
    public static double PANEL_MIN = 0.20;
    public static double PANEL_MAX = 0.85;  // 从数据范围推导


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

    public void shooterHold() {
        leftShooter.setVelocity(800);
        rightShooter.setVelocity(800);
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
        panelTo(Range.clip(panel, PANEL_MIN, PANEL_MAX));
        setShooterVelocity(velocity);
    }

    public void setShooterByDis(double distance) {
        leftShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        rightShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        targetVelocity = f(RPM_A, RPM_B, RPM_C, RPM_D, distance);
        targetPanel = f(PANEL_A, PANEL_B, PANEL_C, PANEL_D, distance);
        setShooter(Range.clip(targetPanel, PANEL_MIN, PANEL_MAX), targetVelocity);
    }
    public void setShooterByDisShow(double distance) {
        targetVelocity = f(RPM_A, RPM_B, RPM_C, RPM_D, distance);
        targetPanel = f(PANEL_A, PANEL_B, PANEL_C, PANEL_D, distance);
    }

    public double calculateIntakePower() {
        double v = targetVelocity;
        if (v < 1540.0) return 1.0;
        if (v < 1720.0) return 0.85;
        if (v < 1800.0) return 0.80;
        if (v < 1880.0) return 0.60;
        return 0.50;
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
