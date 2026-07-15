package org.firstinspires.ftc.teamcode.tele;

import static org.firstinspires.ftc.teamcode.constants.robotConstants.RED_TARGET_X;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.RED_TARGET_Y;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * 红方双操作手 TeleOp。
 *
 * <pre>
 *   1 号手 (gamepad1): 底盘 + 收球/吐球/喂球 + 开关切换 + 里程计复位
 *   2 号手 (gamepad2): 方向键微调（距离/航向）
 * </pre>
 */
@TeleOp(name = "AA TeleOp RED (Dual)", group = "Competition")
@Configurable
public class AA_TeleOp_Red_Dual extends AA_TeleOp_Base {

    public static double RESET_X_IN = 80;
    public static double RESET_Y_IN = 22;
    public static double RESET_H_DEG = -90;

    @Override
    protected void configureMapping(GamepadRole m) {
        m.chassis    = gamepad1;
        m.actions    = gamepad1;
        m.toggles    = gamepad1;
        m.trims      = gamepad2;   // 2号手负责微调
        m.relocalize = gamepad1;
    }

    @Override
    protected double getTargetX() { return RED_TARGET_X; }

    @Override
    protected double getTargetY() { return RED_TARGET_Y; }

    @Override
    protected Pose2D getRelocalizePose() {
        return new Pose2D(DistanceUnit.INCH, RESET_X_IN, RESET_Y_IN,
                          AngleUnit.DEGREES, RESET_H_DEG);
    }

    @Override
    protected String getAllianceName() { return "RED"; }
}
