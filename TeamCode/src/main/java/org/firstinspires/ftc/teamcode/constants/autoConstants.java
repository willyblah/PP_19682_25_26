package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.geometry.Pose;

public class autoConstants {
    public static long TOTAL_SHOOT_TIME = 0;

    public static long AUTO_CLOSE_WAIT_FOR_SHOOT = 0;
    public static double CLOSE_FIRE_DISTANCE = 68;
    public static double CLOSE_FIRE_INTAKE_POWER = 0.8;
    public static double CLOSE_GATE_RUNNING_POWER = 0.5;
    public static double FAR_FIRE_INTAKE_POWER = 0.5;
    public static double CLOSE_HOLD_DISTANCE = CLOSE_FIRE_DISTANCE;
    public static long AUTO_FAR_WAIT_FOR_SHOOT = 800;
    public static double FAR_FIRE_DISTANCE_PRELOAD = 134.4;
    public static double FAR_HOLD_DISTANCE = 123;

    // ==================== BLUE ====================

    public static double BLUE_CLOSE_FIRE_TURRET = -47;
    public static double BLUE_CLOSE_FIRE_TURRET_21 = 0;

    public static double BLUE_INTAKE_START_X = 45;
    public static double BLUE_INTAKE_END_X = 18.8;
    public static double BLUE_INTAKE_FIRST_ROW_Y = 88;
    public static double BLUE_INTAKE_SECOND_ROW_Y = 56;
    public static double BLUE_INTAKE_THIRD_ROW_Y = 36;

    public static Pose BLUE_CLOSE_START = new Pose(23.52, 121.38, Math.toRadians(141.67));
    public static Pose BLUE_CLOSE_SHOOT = new Pose(61.06, 85.1, Math.toRadians(180));
    public static Pose BLUE_CLOSE_SHOOT_21 = new Pose(BLUE_CLOSE_SHOOT.getX(), BLUE_CLOSE_SHOOT.getY(), Math.toRadians(150));
    public static Pose BLUE_CLOSE_SHOOT_CONTROL = new Pose(47.95, 60);
    public static Pose BLUE_CLOSE_SHOOT_PRELOAD = new Pose(BLUE_CLOSE_SHOOT.getX(), BLUE_CLOSE_SHOOT.getY(), BLUE_CLOSE_SHOOT.getHeading());
    public static Pose BLUE_CLOSE_INTAKE_FIRST_CONTROL = new Pose(50, 75);
    public static Pose BLUE_CLOSE_INTAKE_FIRST_CONTROL_21 = new Pose(50, 72);
    public static Pose BLUE_CLOSE_INTAKE_FIRST_END = new Pose(BLUE_INTAKE_END_X - 3, BLUE_INTAKE_FIRST_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_CLOSE_SHOOT_FIRST_ROW = new Pose(59.5, 106, Math.toRadians(150));
    public static Pose BLUE_CLOSE_INTAKE_SECOND_CONTROL = new Pose(47.95, 56);
    public static Pose BLUE_CLOSE_INTAKE_SECOND_END = new Pose(BLUE_INTAKE_END_X - 2, BLUE_INTAKE_SECOND_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_CLOSE_INTAKE_SECOND_GATE = new Pose(15, 63, Math.toRadians(180));
    public static Pose BLUE_CLOSE_SHOOT_SECOND_ROW = new Pose(BLUE_CLOSE_SHOOT.getX(), BLUE_CLOSE_SHOOT.getY(), BLUE_CLOSE_SHOOT.getHeading());
    public static Pose BLUE_CLOSE_INTAKE_THIRD_START = new Pose(BLUE_INTAKE_START_X, BLUE_INTAKE_THIRD_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_CLOSE_INTAKE_THIRD_END = new Pose(BLUE_INTAKE_END_X, BLUE_INTAKE_THIRD_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_CLOSE_SHOOT_THIRD_ROW = new Pose(BLUE_CLOSE_SHOOT.getX(), BLUE_CLOSE_SHOOT.getY(), BLUE_CLOSE_SHOOT.getHeading());
    public static Pose BLUE_CLOSE_INTAKE_GATE = new Pose(22, 59.46, Math.toRadians(143));
    public static Pose BLUE_CLOSE_INTAKE_GATE_CONTROL = new Pose(43, 63.5);
    public static Pose BLUE_CLOSE_INTAKE_GATE_END = new Pose(8, 58, Math.toRadians(155));
    public static Pose BLUE_CLOSE_SHOOT_INTAKE_GATE = new Pose(BLUE_CLOSE_SHOOT.getX(), BLUE_CLOSE_SHOOT.getY(), BLUE_CLOSE_SHOOT.getHeading());
    public static Pose BLUE_CLOSE_PARK = new Pose(55, 65, Math.toRadians(180));

    public static double BLUE_FAR_TURRET = -69;
    public static Pose BLUE_FAR_START = new Pose(59.72, 12.4, Math.toRadians(180.0));
    public static Pose BLUE_FAR_SHOOT = new Pose(BLUE_FAR_START.getX(), BLUE_FAR_START.getY() + 1, BLUE_FAR_START.getHeading());
    public static Pose BLUE_FAR_INTAKE_HP_1_START = new Pose(13, 13, Math.toRadians(180.0));
    public static Pose BLUE_FAR_INTAKE_HP_1_MID = new Pose(12, 25, Math.toRadians(131));
    public static Pose BLUE_FAR_INTAKE_HP_1_END = new Pose(11, 36, Math.toRadians(120));
    public static Pose BLUE_FAR_SHOOT_HP = new Pose(BLUE_FAR_SHOOT.getX(), BLUE_FAR_SHOOT.getY(), BLUE_FAR_SHOOT.getHeading());
    public static Pose BLUE_FAR_SHOOT_TUNNEL = new Pose(BLUE_FAR_SHOOT.getX(), BLUE_FAR_SHOOT.getY(), BLUE_FAR_SHOOT.getHeading());
    public static Pose BLUE_FAR_PARK = new Pose(37.8, 16.8, Math.toRadians(180.0));
    public static Pose BLUE_FAR_INTAKE_THIRD_CONTROL = new Pose(70, 38);
    public static Pose BLUE_FAR_INTAKE_THIRD_END = new Pose(14.4, 38, Math.toRadians(180.0));
    public static Pose BLUE_FAR_INTAKE_TUNNEL_START = new Pose(54.0, 26.5, Math.toRadians(180.0));
    public static Pose BLUE_FAR_INTAKE_TUNNEL_END = new Pose(12.0, 26.5, Math.toRadians(165.0));

    // ==================== RED ====================

    public static double RED_CLOSE_FIRE_TURRET = 47;
    public static double RED_CLOSE_FIRE_TURRET_21 = 85;


    public static double RED_INTAKE_START_X = 99;
    public static double RED_INTAKE_END_X = 125.2;
    public static double RED_INTAKE_FIRST_ROW_Y = 85.9;
    public static double RED_INTAKE_SECOND_ROW_Y = 55;
    public static double RED_INTAKE_THIRD_ROW_Y = 36;

    public static Pose RED_CLOSE_START = new Pose(120.48, 121.38, Math.toRadians(38.33));
    public static Pose RED_CLOSE_SHOOT = new Pose(82.94, 85.1, Math.toRadians(0));
    public static Pose RED_CLOSE_SHOOT_21 = new Pose(RED_CLOSE_SHOOT.getX(), RED_CLOSE_SHOOT.getY(), Math.toRadians(-45));

    public static Pose RED_CLOSE_SHOOT_CONTROL = new Pose(96.05, 60);
    public static Pose RED_CLOSE_SHOOT_PRELOAD = new Pose(82.94, 85.1, Math.toRadians(0));
    public static Pose RED_CLOSE_SHOOT_PRELOAD_21 = new Pose(RED_CLOSE_SHOOT_21.getX(), RED_CLOSE_SHOOT.getY(), Math.toRadians(-45));

    public static Pose RED_CLOSE_INTAKE_FIRST_CONTROL = new Pose(94, 77);
    public static Pose RED_CLOSE_INTAKE_FIRST_CONTROL_21 = new Pose(94, 72);

    public static Pose RED_CLOSE_INTAKE_FIRST_END = new Pose(129, 85.9, Math.toRadians(0));
    public static Pose RED_CLOSE_SHOOT_FIRST_ROW = new Pose(84.5, 106, Math.toRadians(30));

    public static Pose RED_CLOSE_INTAKE_SECOND_CONTROL = new Pose(96.05, 55);
    public static Pose RED_CLOSE_INTAKE_SECOND_END = new Pose(128.2, 55, Math.toRadians(0));
    public static Pose RED_CLOSE_SHOOT_SECOND_ROW = new Pose(82.94, 85.1, Math.toRadians(0));
    public static Pose RED_CLOSE_INTAKE_SECOND_GATE = new Pose(128, 63, Math.toRadians(0));

    public static Pose RED_CLOSE_INTAKE_THIRD_START = new Pose(99, 37, Math.toRadians(0));
    public static Pose RED_CLOSE_INTAKE_THIRD_END = new Pose(125.2, 37, Math.toRadians(0));
    public static Pose RED_CLOSE_SHOOT_THIRD_ROW = new Pose(82.94, 85.1, Math.toRadians(0));

    public static Pose RED_CLOSE_INTAKE_GATE = new Pose(121, 59.46, Math.toRadians(37));
    public static Pose RED_CLOSE_INTAKE_GATE_CONTROL = new Pose(101, 63.5);
    public static Pose RED_CLOSE_INTAKE_GATE_END = new Pose(132, 57.8, Math.toRadians(32));
    public static Pose RED_CLOSE_SHOOT_INTAKE_GATE = new Pose(82.94, 85.1, Math.toRadians(0));

    public static Pose RED_CLOSE_PARK = new Pose(89, 65, Math.toRadians(0));

    public static double RED_FAR_TURRET = 69;

    public static Pose RED_FAR_START = new Pose(84.28, 12.4, Math.toRadians(0));
    public static Pose RED_FAR_SHOOT = new Pose(84.28, 13.4, Math.toRadians(0));

    public static Pose RED_FAR_INTAKE_HP_1_START = new Pose(131, 13, Math.toRadians(0));
    public static Pose RED_FAR_INTAKE_HP_1_MID = new Pose(132, 25, Math.toRadians(49));
    public static Pose RED_FAR_INTAKE_HP_1_END = new Pose(133, 36, Math.toRadians(60));

    public static Pose RED_FAR_SHOOT_HP = new Pose(84.28, 13.4, Math.toRadians(0));
    public static Pose RED_FAR_SHOOT_TUNNEL = new Pose(84.28, 13.4, Math.toRadians(0));

    public static Pose RED_FAR_PARK = new Pose(106.2, 16.8, Math.toRadians(0));

    public static Pose RED_FAR_INTAKE_THIRD_CONTROL = new Pose(60, 35.7);
    public static Pose RED_FAR_INTAKE_THIRD_END = new Pose(129.6, 36, Math.toRadians(0));

    public static Pose RED_FAR_INTAKE_TUNNEL_START = new Pose(90.0, 26.5, Math.toRadians(0));
    public static Pose RED_FAR_INTAKE_TUNNEL_END = new Pose(132.0, 26.5, Math.toRadians(15));
}