package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.robotConstants.TURRET_FULL_RANGE_DEGREE;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.TURRET_FULL_RANGE_ENCODER;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * 转塔"编码器tick ↔ 物理角度"标定工具。
 *
 * 用法：
 *   1. 在转塔上贴一个指针标记(比如一条胶带箭头)，底盘上固定一张打印的量角器/靠一把角尺。
 *   2. 把转塔点动(用 右保险杠=正向点动 / 左保险杠=反向点动，都是低功率慢速)到第一个参考角度
 *      (比如物理左极限，或者你觉得方便量角的任意一个位置)，按 X 键把编码器清零，
 *      这个位置就是你的"0度参考点"。
 *   3. 继续点动转塔到第二个参考角度（建议尽量转到另一侧物理极限，角度差越大，
 *      标定出来的比例误差越小），用量角器实测这两点之间转过的真实角度，记下来。
 *   4. 同时看屏幕上 "rawTicks" 这个数字，这就是转过这个角度对应的编码器计数。
 *   5. 算 (tick每度) = rawTicks / 实测角度。
 *      然后 constants/robotConstants.java 里可以直接填:
 *        TURRET_FULL_RANGE_DEGREE  = 实测角度
 *        TURRET_FULL_RANGE_ENCODER = rawTicks
 *      (这两个数字只是一个比值，不一定非要填物理满量程，只要角度和ticks是同一次测量配对的就行；
 *       但建议尽量测大角度，测量误差占比更小)
 *
 * 安全提示: 点动功率固定为低功率(JOG_POWER)，松开按键立刻停止，不会自己乱转。
 */

@TeleOp(name = "Turret Tick Calibration", group = "Tuning")
public class TurretTickCalibration extends LinearOpMode {

    private static final double JOG_POWER = 0.15; // 点动功率，故意给低一点，方便精确对准角度

    private final Robot robot = new Robot();
    private JoinedTelemetry joinedTele;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        joinedTele.addData("说明", "右保险杠=正向点动 左保险杠=反向点动 X=清零当前位置为0度参考点");
        joinedTele.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                robot.shooter.turret.setPower(JOG_POWER);
            } else if (gamepad1.left_bumper) {
                robot.shooter.turret.setPower(-JOG_POWER);
            } else {
                robot.shooter.turret.setPower(0.0);
            }

            if (gamepad1.xWasPressed()) {
                robot.shooter.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.shooter.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            int rawTicks = robot.shooter.turret.getCurrentPosition();
            double estimatedDeg = rawTicks / TURRET_FULL_RANGE_ENCODER * TURRET_FULL_RANGE_DEGREE;

            joinedTele.addData("rawTicks(核心数据，记这个)", rawTicks);
            joinedTele.addData("估算角度(按旧比例换算，仅供参考对照)", estimatedDeg);
            joinedTele.addData("旧比例", TURRET_FULL_RANGE_ENCODER + " ticks / " + TURRET_FULL_RANGE_DEGREE + " 度");
            joinedTele.update();
        }
    }
}
