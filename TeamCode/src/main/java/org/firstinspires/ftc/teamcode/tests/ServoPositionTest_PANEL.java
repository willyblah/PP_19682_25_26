package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.L_GATE;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.PANEL;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp
public class ServoPositionTest_PANEL extends LinearOpMode {
    Drivetrain drivetrain = new Drivetrain();
    private double x, y, rx, p;
    private String[] names = {PANEL, L_GATE};
    private int index = 0, servoNum = names.length;
    private Servo[] servos = new Servo[servoNum];
    private double[] poses = new double[servoNum];
    private boolean upHold = false, downHold = false;
    JoinedTelemetry joinedTele;
    GamepadManager g1;

    @Override
    public void runOpMode() throws InterruptedException {
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        g1 = PanelsGamepad.INSTANCE.getFirstManager();
        for (int i = 0; i < servoNum; i++) {
            servos[i] = hardwareMap.get(Servo.class, names[i]);
            poses[i] = 0.5;
        }
        drivetrain.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            g1.asCombinedFTCGamepad(gamepad1);
            drivetrain.drive(g1, 0);
//            leftFront.setPower(gamepad1.x ? 1 : 0);
//            leftBack.setPower(gamepad1.a ? 1 : 0);
//            rightFront.setPower(gamepad1.y ? 1 : 0);
//            rightBack.setPower(gamepad1.b ? 1 : 0);

            if (g1.getDpadUp() && !upHold && index < servoNum - 1) {
                upHold = true;
                index++;
            } else if (!g1.getDpadUp()) {
                upHold = false;
            }
            if (g1.getDpadDown() && !downHold && index > 0) {
                downHold = true;
                index--;
            } else if (!g1.getDpadDown()) {
                downHold = false;
            }

            poses[index] = Math.min(1.0, Math.max(0.0, (g1.getR2() - g1.getL2()) / 500 + poses[index]));

            if (g1.getR1()) {
//                servos[index].setPwmEnable();
                servos[index].setPosition(poses[index]);
            }

            if (g1.getL1()) {
                servos[index].setPosition(0);
//                servos[index].setPwmDisable();
            }

            joinedTele.addData("Current Servo", String.valueOf(names[index]));
            for (int i = 0; i < servoNum; i++) {
                joinedTele.addData(i + " " + names[i], String.valueOf(poses[i]));
            }
            joinedTele.update();
        }
    }
}
