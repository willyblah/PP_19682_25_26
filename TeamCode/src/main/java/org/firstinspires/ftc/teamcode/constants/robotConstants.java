package org.firstinspires.ftc.teamcode.constants;

public class robotConstants {
    public static double TURRET_FULL_RANGE_DEGREE = 320.0; // 175
    public static double TURRET_FULL_RANGE_ENCODER = 1050;// 566
    public static double TURRET_ABS_RANGE_DEGREE = 160; // 175

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

    public static double L_GATE_OPEN = 0.554;
    public static double L_GATE_CLOSE = 0.403;
    public static double R_GATE_OPEN = 0.5;
    public static double R_GATE_CLOSE = 0.66;
    public static double CURVE_OPEN = 0.639; // 发射
    public static double CURVE_CLOSE = 0.94; // 关闭发射

    public static volatile double autoEndX = 72;
    public static volatile double autoEndY = 72;
    public static volatile double autoEndH = Math.PI / 2.0;

    public static volatile double BLUE_TARGET_X = 136.5;
    public static volatile double BLUE_TARGET_Y = 136.0;
    public static volatile double RED_TARGET_X = 136.5;
    public static volatile double RED_TARGET_Y = 8.0;

    public static volatile double teleOpTargetX;
    public static volatile double teleOpTargetY;
}
