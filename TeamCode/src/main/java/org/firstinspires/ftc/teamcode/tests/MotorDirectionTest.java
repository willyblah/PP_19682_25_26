package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.INTAKE;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_BACK;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_FRONT;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_BACK;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_FRONT;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.TURRET;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
public class MotorDirectionTest extends LinearOpMode {
    private String[] names = {LEFT_FRONT, LEFT_BACK, RIGHT_FRONT, RIGHT_BACK, INTAKE, TURRET};
    private int index = 0, motorNum = names.length;
    private DcMotorEx[] motors = new DcMotorEx[motorNum];
    private double[] poses = new double[motorNum];
    private boolean upHold = false, downHold = false;

    @Override
    public void runOpMode() throws InterruptedException {
        for (int i = 0; i < motorNum; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, names[i]);
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !upHold && index < motorNum - 1) {
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

            motors[index].setPower(gamepad1.y ? gamepad1.right_trigger : gamepad1.a ? -gamepad1.right_trigger : 0);

            telemetry.addData("Power", (gamepad1.a ? "-" : "") + gamepad1.left_trigger);
            telemetry.addData("Current Motor", names[index]);
            for (int i = 0; i < motorNum; i++) {
                telemetry.addData(i + " " + names[i], motors[i].getCurrentPosition());
            }
            telemetry.update();
        }
    }
}
