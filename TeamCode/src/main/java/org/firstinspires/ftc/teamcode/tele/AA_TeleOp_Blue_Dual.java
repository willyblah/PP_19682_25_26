package org.firstinspires.ftc.teamcode.tele;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * 蓝方 TeleOp。目标点/复位坐标跟 A_2_AA_AS_Blue.java 完全一致。
 */
@TeleOp(name = "AA TeleOp BLUE (Dual)", group = "Competition")
@Configurable
public class AA_TeleOp_Blue_Dual extends AA_TeleOp_Base {

    public static double TARGET_X = 136.5;
    public static double TARGET_Y = 136;

    public static double RESET_X_IN = 80;
    public static double RESET_Y_IN = 124;
    public static double RESET_H_DEG = 90;

    @Override
    protected double getTargetX() { return TARGET_X; }

    @Override
    protected double getTargetY() { return TARGET_Y; }

    @Override
    protected Pose2D getRelocalizePose() {
        return new Pose2D(DistanceUnit.INCH, RESET_X_IN, RESET_Y_IN,
                          AngleUnit.DEGREES, RESET_H_DEG);
    }

    @Override
    protected String getAllianceName() { return "BLUE"; }
}
