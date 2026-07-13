package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.CURVE;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.L_GATE;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_BACK;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_FRONT;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.PANEL;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_BACK;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_FRONT;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.R_GATE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoPositionTest extends LinearOpMode {
    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;

    private double x, y, rx, p;
    private String[] names = {L_GATE, R_GATE, PANEL, CURVE};
    private int index = 0, servoNum = names.length;
    private Servo[] servos = new Servo[servoNum];
    private double[] poses = new double[servoNum];
    private boolean upHold = false, downHold = false;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT);
        leftBack = hardwareMap.get(DcMotorEx.class, LEFT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT);
        rightBack = hardwareMap.get(DcMotorEx.class, RIGHT_BACK);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
//        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
//        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        for (int i = 0; i < servoNum; i++) {
            servos[i] = hardwareMap.get(Servo.class, names[i]);
            poses[i] = 0.5;
        }

        waitForStart();
        while (opModeIsActive()) {
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            if (gamepad1.left_bumper) p = 0.3;
            else p = 1;
            leftFront.setPower((y + x + rx) * p);
            leftBack.setPower((y - x + rx) * p);
            rightFront.setPower((y - x - rx) * p);
            rightBack.setPower((y + x - rx) * p);

//            leftFront.setPower(gamepad1.x ? 1 : 0);
//            leftBack.setPower(gamepad1.a ? 1 : 0);
//            rightFront.setPower(gamepad1.y ? 1 : 0);
//            rightBack.setPower(gamepad1.b ? 1 : 0);

            if (gamepad1.dpad_up && !upHold && index < servoNum - 1) {
                upHold = true;
                index++;
            } else if (!gamepad1.dpad_up) {
                upHold = false;
            }
            if (gamepad1.dpad_down && !downHold && index > 0) {
                downHold = true;
                index--;
            } else if (!gamepad1.dpad_down) {
                downHold = false;
            }

//            poses[index] = Math.min(1.0, Math.max(0.0, (gamepad1.right_trigger - gamepad1.left_trigger) / 1300 + poses[index]));
            if(gamepad1.aWasPressed()) poses[index] -= 0.01;
            if(gamepad1.yWasPressed()) poses[index] += 0.01;
            poses[index] = Math.min(1.0, Math.max(0.0, poses[index]));

            if (gamepad1.right_bumper) {
//                servos[index].setPwmEnable();
                servos[index].setPosition(poses[index]);
            }

            if (gamepad1.left_bumper) {
                servos[index].setPosition(0);
//                servos[index].setPwmDisable();
            }

            telemetry.addData("Current Servo", String.valueOf(names[index]));
            for (int i = 0; i < servoNum; i++) {
                telemetry.addData(i + " " + names[i], String.valueOf(poses[i]));
            }
            telemetry.update();
        }
    }
}
