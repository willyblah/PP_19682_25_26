package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.geometry.Pose;

public class autoConstants {
    public static long TOTAL_SHOOT_TIME = 0;
    public static long INTAKE_TIME = 300; // 毫秒

    public static long AUTO_CLOSE_WAIT_FOR_SHOOT = 0;
    public static double CLOSE_FIRE_DISTANCE = 75.0;
    public static double CLOSE_FIRE_DISTANCE_LAST = 60.0;
    public static double CLOSE_FIRE_INTAKE_POWER = 0.8;
    public static double CLOSE_HOLD_DISTANCE = CLOSE_FIRE_DISTANCE; // 40.0;

    public static double RED_CLOSE_FIRE_TURRET = 45;
    public static double RED_CLOSE_FIRE_TURRET_LAST = 0;

    public static double RED_INTAKE_START_X = 95;
    public static double RED_INTAKE_END_X = 118;
    public static double RED_INTAKE_FIRST_ROW_Y = 84;
    public static double RED_INTAKE_SECOND_ROW_Y = 59.5;
    public static double RED_INTAKE_THIRD_ROW_Y = 36;

    public static Pose RED_CLOSE_START = new Pose(122.406, 119.482, Math.toRadians(38.49));
    public static Pose RED_CLOSE_SHOOT = new Pose(99.5, 99.5, Math.toRadians(0));
    public static Pose RED_CLOSE_SHOOT_CONTROL = new Pose(88, 58);
    public static Pose RED_CLOSE_SHOOT_PRELOAD = new Pose(RED_CLOSE_SHOOT.getX(), RED_CLOSE_SHOOT.getY(), RED_CLOSE_SHOOT.getHeading());
    public static Pose RED_CLOSE_INTAKE_FIRST_CONTROL = new Pose(94, 78);
    public static Pose RED_CLOSE_INTAKE_FIRST_END = new Pose(RED_INTAKE_END_X, RED_INTAKE_FIRST_ROW_Y, Math.toRadians(0));
    public static Pose RED_CLOSE_SHOOT_FIRST_ROW = new Pose(84.5, 106, Math.toRadians(30));
    public static Pose RED_CLOSE_INTAKE_SECOND_CONTROL = new Pose(86, 52);
    public static Pose RED_CLOSE_INTAKE_SECOND_END = new Pose(RED_INTAKE_END_X, RED_INTAKE_SECOND_ROW_Y, Math.toRadians(0));
    public static Pose RED_CLOSE_SHOOT_SECOND_ROW = new Pose(RED_CLOSE_SHOOT.getX(), RED_CLOSE_SHOOT.getY(), RED_CLOSE_SHOOT.getHeading());
    public static Pose RED_CLOSE_INTAKE_THIRD_START = new Pose(RED_INTAKE_START_X, RED_INTAKE_THIRD_ROW_Y, Math.toRadians(0));
    public static Pose RED_CLOSE_INTAKE_THIRD_END = new Pose(RED_INTAKE_END_X, RED_INTAKE_THIRD_ROW_Y, Math.toRadians(0));
    public static Pose RED_CLOSE_SHOOT_THIRD_ROW = new Pose(RED_CLOSE_SHOOT.getX(), RED_CLOSE_SHOOT.getY(), RED_CLOSE_SHOOT.getHeading());
    public static Pose RED_CLOSE_INTAKE_GATE = new Pose(129.5, 58.5, Math.toRadians(32.5));
    public static Pose RED_CLOSE_SHOOT_INTAKE_GATE = new Pose(RED_CLOSE_SHOOT.getX(), RED_CLOSE_SHOOT.getY(), RED_CLOSE_SHOOT.getHeading());
    public static Pose RED_CLOSE_SHOOT_THIRD_ROW_CONTROL = new Pose(85, 66);

    public static double BLUE_CLOSE_FIRE_TURRET = -45;
    public static double BLUE_CLOSE_FIRE_TURRET_LAST = 0;

    public static double BLUE_INTAKE_START_X = 45;
    public static double BLUE_INTAKE_END_X = 18.8;
    public static double BLUE_INTAKE_FIRST_ROW_Y = 85.9;
    public static double BLUE_INTAKE_SECOND_ROW_Y = 55;
    public static double BLUE_INTAKE_THIRD_ROW_Y = 36;

    public static Pose BLUE_CLOSE_START = new Pose(23.52, 121.38, Math.toRadians(141.67));
    public static Pose BLUE_CLOSE_SHOOT = new Pose(61.06, 85.1, Math.toRadians(180));
    public static Pose BLUE_CLOSE_SHOOT_CONTROL = new Pose(47.95, 60);
    public static Pose BLUE_CLOSE_SHOOT_PRELOAD = new Pose(BLUE_CLOSE_SHOOT.getX(), BLUE_CLOSE_SHOOT.getY(), BLUE_CLOSE_SHOOT.getHeading());
    public static Pose BLUE_CLOSE_INTAKE_FIRST_CONTROL = new Pose(50, 78);
    public static Pose BLUE_CLOSE_INTAKE_FIRST_END = new Pose(BLUE_INTAKE_END_X - 3, BLUE_INTAKE_FIRST_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_CLOSE_SHOOT_FIRST_ROW = new Pose(59.5, 106, Math.toRadians(150));
    public static Pose BLUE_CLOSE_INTAKE_SECOND_CONTROL = new Pose(47.95, 55);
    public static Pose BLUE_CLOSE_INTAKE_SECOND_END = new Pose(BLUE_INTAKE_END_X - 3, BLUE_INTAKE_SECOND_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_CLOSE_SHOOT_SECOND_ROW = new Pose(BLUE_CLOSE_SHOOT.getX(), BLUE_CLOSE_SHOOT.getY(), BLUE_CLOSE_SHOOT.getHeading());
    public static Pose BLUE_CLOSE_INTAKE_THIRD_START = new Pose(BLUE_INTAKE_START_X, BLUE_INTAKE_THIRD_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_CLOSE_INTAKE_THIRD_END = new Pose(BLUE_INTAKE_END_X, BLUE_INTAKE_THIRD_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_CLOSE_SHOOT_THIRD_ROW = new Pose(BLUE_CLOSE_SHOOT.getX(), BLUE_CLOSE_SHOOT.getY(), BLUE_CLOSE_SHOOT.getHeading());
    public static Pose BLUE_CLOSE_INTAKE_GATE = new Pose(13.27, 59.46, Math.toRadians(155.5));
    public static Pose BLUE_CLOSE_INTAKE_GATE_CONTROL = new Pose(45.5, 63.5);
    public static Pose BLUE_CLOSE_INTAKE_GATE_END = new Pose(15.27, 55.46, Math.toRadians(150.5));
    public static Pose BLUE_CLOSE_SHOOT_INTAKE_GATE = new Pose(BLUE_CLOSE_SHOOT.getX(), BLUE_CLOSE_SHOOT.getY(), BLUE_CLOSE_SHOOT.getHeading());
    public static Pose BLUE_CLOSE_PARK = new Pose(55, 65, Math.toRadians(180));

    public static long AUTO_FAR_WAIT_FOR_SHOOT = 400;
    public static double FAR_FIRE_DISTANCE = 123;
    public static double FAR_FIRE_DISTANCE_PRELOAD = 134.4;
    public static double FAR_HOLD_DISTANCE = 123;
    public static double RED_FAR_TURRET = 65;
    public static double RED_FAR_TURRET_PRELOAD = 65;

    public static Pose RED_FAR_START = new Pose(85.5, 7.4, Math.toRadians(0));
    public static Pose RED_FAR_SHOOT = new Pose(85.0, 17, Math.toRadians(0));
    public static Pose RED_FAR_INTAKE_HP_1_START = new Pose(120, 8, Math.toRadians(0));
    public static Pose RED_FAR_INTAKE_HP_1_MID = new Pose(132.5, 9, Math.toRadians(-20));
    public static Pose RED_FAR_INTAKE_HP_1_END = new Pose(131.5, 15, Math.toRadians(20));
    public static Pose RED_FAR_SHOOT_HP = new Pose(RED_FAR_SHOOT.getX(), RED_FAR_SHOOT.getY(), RED_FAR_SHOOT.getHeading());
    public static Pose RED_FAR_SHOOT_TUNNEL = new Pose(RED_FAR_SHOOT.getX(), RED_FAR_SHOOT.getY(), RED_FAR_SHOOT.getHeading());
    public static Pose RED_FAR_PARK = new Pose(96, 27, Math.toRadians(0));
    public static Pose RED_FAR_INTAKE_THIRD_CONTROL = new Pose(86, 38);
    public static Pose RED_FAR_INTAKE_THIRD_END = new Pose(118.0, 34, Math.toRadians(0.0));
    public static Pose RED_FAR_INTAKE_TUNNEL_START = new Pose(90.0, 26.5, Math.toRadians(0.0));
    public static Pose RED_FAR_INTAKE_TUNNEL_END = new Pose(132.0, 26.5, Math.toRadians(15.0));

    public static double BLUE_FAR_TURRET = -69;
    public static Pose BLUE_FAR_START = new Pose(12.4, 59.72, Math.toRadians(180.0));
    public static Pose BLUE_FAR_SHOOT = new Pose(BLUE_FAR_START.getX(), BLUE_FAR_START.getY(), BLUE_FAR_START.getHeading());
    public static Pose BLUE_FAR_INTAKE_HP_1_START = new Pose(19.1, 27.5, Math.toRadians(180.0));
    public static Pose BLUE_FAR_INTAKE_HP_1_MID = new Pose(21.3, 13.8, Math.toRadians(111.2));
    public static Pose BLUE_FAR_INTAKE_HP_1_END = new Pose(34.2, 10.2, Math.toRadians(99.5));
    public static Pose BLUE_FAR_SHOOT_HP = new Pose(BLUE_FAR_SHOOT.getX(), BLUE_FAR_SHOOT.getY(), BLUE_FAR_SHOOT.getHeading());
    public static Pose BLUE_FAR_SHOOT_TUNNEL = new Pose(BLUE_FAR_SHOOT.getX(), BLUE_FAR_SHOOT.getY(), BLUE_FAR_SHOOT.getHeading());
    public static Pose BLUE_FAR_PARK = new Pose(48.0, 27.0, Math.toRadians(180.0));
    public static Pose BLUE_FAR_INTAKE_THIRD_CONTROL = new Pose(39.7, 46.5);
    public static Pose BLUE_FAR_INTAKE_THIRD_END = new Pose(39, 14.4, Math.toRadians(180.0));
    public static Pose BLUE_FAR_INTAKE_TUNNEL_START = new Pose(54.0, 26.5, Math.toRadians(180.0));
    public static Pose BLUE_FAR_INTAKE_TUNNEL_END = new Pose(12.0, 26.5, Math.toRadians(165.0));
}
