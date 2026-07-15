package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.*;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotorEx intake;
    private Servo lGate, rGate, curve;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, INTAKE);
        lGate = hardwareMap.get(Servo.class, L_GATE);
        rGate = hardwareMap.get(Servo.class, R_GATE);
        curve = hardwareMap.get(Servo.class, CURVE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void intakeIn() {
        intake.setPower(0.8);
    }

    public void intakeIn(double power) {
        intake.setPower(power);
    }

    public void intakeFire(double power) {
        intake.setPower(power);
    }

    public void intakeInSlow() {
        intake.setPower(0.35);
    }

    public void intakeOut() {
        intake.setPower(-1);
    }

    public void intakeOut(double power) {
        intake.setPower(-power);
    }

    public void intakeStop() {
        intake.setPower(0);
    }

    public void gateOpen() {
        lGate.setPosition(L_GATE_OPEN);
        rGate.setPosition(R_GATE_OPEN);
        curve.setPosition(CURVE_OPEN);
    }

    public void gateClose() {
        lGate.setPosition(L_GATE_CLOSE);
        rGate.setPosition(R_GATE_CLOSE);
        curve.setPosition(CURVE_CLOSE);
    }

    public void setCurve(double position) {
        curve.setPosition(position);
    }
}
