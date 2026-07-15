package org.firstinspires.ftc.teamcode.tele;

import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndH;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndY;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.targetVelocity;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.AutoAimSubsystem_MK2;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * ============================================================================
 * AA_TeleOp_Base —— 自瞄 TeleOp 基类（重构版）
 * ============================================================================
 *
 * <p>所有操作/瞄准逻辑集中在这里。子类只需提供 5 项信息：
 * <ol>
 *   <li>球门目标点坐标（{@link #getTargetX}/{@link #getTargetY}）</li>
 *   <li>里程计复位基准点（{@link #getRelocalizePose}）</li>
 *   <li>联盟名称（{@link #getAllianceName}）</li>
 *   <li>手柄功能分配（{@link #configureMapping}）</li>
 * </ol>
 *
 * <h3>手柄映射说明</h3>
 * 子类通过 {@link #configureMapping(GamepadRole)} 决定每个功能分配到哪个手柄：
 *
 * <pre>{@code
 *   // 单操作手：全部用 gamepad1
 *   m.chassis    = gamepad1;  // 底盘摇杆
 *   m.actions    = gamepad1;  // 扳机收球/吐球、RB喂球
 *   m.toggles    = gamepad1;  // LB炮塔开关、Y移动发射、A手动测距
 *   m.trims      = gamepad1;  // 方向键距离/航向微调
 *   m.relocalize = gamepad1;  // Start 里程计复位
 *
 *   // 双操作手：1号手底盘+收球，2号手微调
 *   m.chassis    = gamepad1;
 *   m.actions    = gamepad1;
 *   m.toggles    = gamepad1;
 *   m.trims      = gamepad2;
 *   m.relocalize = gamepad1;
 * }</pre>
 */
@Configurable
public abstract class AA_TeleOp_Base extends LinearOpMode {

    // ========================================================================
    // 手柄功能映射 —— 子类通过 configureMapping() 分配手柄
    // ========================================================================

    public static class GamepadRole {
        /** 底盘控制（左/右摇杆） */
        public Gamepad chassis;
        /** 收球/吐球/喂球（扳机 + RB） */
        public Gamepad actions;
        /** 模式切换（LB / Y / A） */
        public Gamepad toggles;
        /** 方向键微调（dpad 距离/航向） */
        public Gamepad trims;
        /** 里程计复位（Start） */
        public Gamepad relocalize;
    }

    /**
     * 子类在此方法中把 gamepad1 / gamepad2 分配给各个功能。
     * <p>在 {@link #runOpMode()} 初始化阶段调用一次。
     */
    protected abstract void configureMapping(GamepadRole m);

    /** 当前实例的手柄映射（在 runOpMode 中初始化后有效）。 */
    protected GamepadRole mapping;

    // ========================================================================
    // @Configurable 可调参数
    // ========================================================================

    public static double drivePower              = 1.0;
    public static double yawCorrection           = 0.0;
    public static double velocityCorrection      = 0.0;
    public static double distanceCorrection      = 0.0;
    public static boolean shootOnTheMoveDefault  = true;
    public static boolean manualDistanceMode     = false;
    public static double manualDistance          = 60.0;
    public static double brakeStickThreshold     = 0.15;

    private static final double MAX_YAW_CORRECTION_DEG     = 15.0;
    private static final double MAX_DISTANCE_CORRECTION_IN = 24.0;

    // ========================================================================
    // 子系统
    // ========================================================================
    protected final Robot robot = new Robot();
    protected final AutoAimSubsystem_MK2 autoAim = new AutoAimSubsystem_MK2();
    protected JoinedTelemetry joinedTele;

    // ========================================================================
    // 运行时状态
    // ========================================================================
    private boolean shooterOn = false;
    private boolean shootOnTheMove;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ========================================================================
    // 子类必须提供的信息
    // ========================================================================

    protected abstract double getTargetX();
    protected abstract double getTargetY();
    protected abstract Pose2D getRelocalizePose();
    protected abstract String getAllianceName();

    // ========================================================================
    // 主流程
    // ========================================================================
    @Override
    public void runOpMode() throws InterruptedException {
        // ---- 初始化 ----
        robot.init(hardwareMap);
        autoAim.init(hardwareMap);
        shootOnTheMove = shootOnTheMoveDefault;
        mapping = new GamepadRole();
        configureMapping(mapping);

        // 校验：所有映射必须非空
        validateMapping();

        resetToAutoEndPose();

        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        joinedTele.addData("alliance", getAllianceName());
        joinedTele.addData("targetX", getTargetX());
        joinedTele.addData("targetY", getTargetY());
        joinedTele.addData("提示", "按 START 前请确认机器人已放置在正确起始位置");
        joinedTele.update();

        waitForStart();
        resetToAutoEndPose();
        loopTimer.reset();

        while (opModeIsActive()) {
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            // 1. 底盘
            robot.drivetrain.drive(mapping.chassis, drivePower);

            // 2. 进球/吐球/喂球
            handleIntake();

            // 3. 模式切换
            handleToggles();

            // 4. 方向键微调
            handleTrims();

            // 5. 里程计复位
            handleFieldRelocalize();

            // 6. 读取位姿 → 自瞄
            Pose2D current = robot.drivetrain.getPosition();
            double rx  = current.getX(DistanceUnit.INCH);
            double ry  = current.getY(DistanceUnit.INCH);
            double hdg = current.getHeading(AngleUnit.DEGREES);
            double vx  = robot.drivetrain.pinPoint.getVelX(DistanceUnit.INCH);
            double vy  = robot.drivetrain.pinPoint.getVelY(DistanceUnit.INCH);
            double omg = robot.drivetrain.pinPoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
            double mDist = Math.max(0.0, manualDistance + distanceCorrection);
            boolean braking = Math.hypot(mapping.chassis.left_stick_x,
                                         mapping.chassis.left_stick_y) < brakeStickThreshold;

            AutoAimSubsystem_MK2.SHOT_DIST_OFFSET = distanceCorrection;
            AutoAimSubsystem_MK2.TurretCommand cmd = autoAim.update(
                    rx, ry, vx, vy, hdg, omg,
                    getTargetX(), getTargetY(),
                    manualDistanceMode, mDist,
                    shootOnTheMove, braking, yawCorrection
            );

            // 7. 飞轮 + 挡球板
            applyShooterCommand(cmd);

            // 8. 遥测
            updateTelemetry(rx, ry, hdg, vx, vy, omg, cmd, loopMs);
        }

        autoAim.stop();
        robot.shooter.shooterStop();
        robot.intake.intakeStop();
    }

    // ========================================================================
    // 各处理函数
    // ========================================================================

    private void resetToAutoEndPose() {
        robot.drivetrain.pinPoint.setPosition(new Pose2D(
                DistanceUnit.INCH, autoEndY, 144.0 - autoEndX,
                AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
    }

    private void handleFieldRelocalize() {
        if (mapping.relocalize.startWasPressed()) {
            robot.drivetrain.pinPoint.setPosition(getRelocalizePose());
        }
    }

    /**
     * 进球/吐球/喂球。
     * <ul>
     *   <li>右扳机(>0.1) → 收球，同时关炮塔</li>
     *   <li>左扳机(>0.1) → 吐球，功率=扳机行程</li>
     *   <li>右保险杠(按住) → 喂球给炮塔，功率动态</li>
     *   <li>都没按 → 停止</li>
     * </ul>
     */
    private void handleIntake() {
        Gamepad g = mapping.actions;
        if (g.right_trigger > 0.1) {
            robot.intake.intakeIn();
            shooterOn = false;
        } else if (g.left_trigger > 0.1) {
            robot.intake.intakeOut(g.left_trigger);
        } else if (g.right_bumper) {
            robot.intake.intakeIn(robot.shooter.calculateIntakePower());
        } else {
            robot.intake.intakeStop();
        }
    }

    /** 边沿触发，按一次翻转。 */
    private void handleToggles() {
        Gamepad g = mapping.toggles;
        if (g.leftBumperWasPressed()) shooterOn        = !shooterOn;
        if (g.yWasPressed())          shootOnTheMove    = !shootOnTheMove;
        if (g.aWasPressed())          manualDistanceMode = !manualDistanceMode;
    }

    /** 方向键微调，带安全限幅。 */
    private void handleTrims() {
        Gamepad g = mapping.trims;
        if (g.dpadUpWasPressed())    distanceCorrection += 2.0;
        if (g.dpadDownWasPressed())  distanceCorrection -= 2.0;
        if (g.dpadLeftWasPressed())  yawCorrection      += 1.0;
        if (g.dpadRightWasPressed()) yawCorrection      -= 1.0;

        distanceCorrection = Range.clip(distanceCorrection,
                -MAX_DISTANCE_CORRECTION_IN, MAX_DISTANCE_CORRECTION_IN);
        yawCorrection = Range.clip(yawCorrection,
                -MAX_YAW_CORRECTION_DEG, MAX_YAW_CORRECTION_DEG);
    }

    /**
     * 炮塔开启 + 有有效目标 → 开挡球板、飞轮满速。
     * 否则 → 关挡球板、飞轮低速待机。
     */
    private void applyShooterCommand(AutoAimSubsystem_MK2.TurretCommand cmd) {
        if (shooterOn && cmd.hasTarget) {
            robot.intake.gateOpen();
            targetVelocity = cmd.targetRpm + velocityCorrection;
            robot.shooter.setShooterVelocity(targetVelocity);
        } else {
            robot.intake.gateClose();
            robot.shooter.shooterHold();
        }
    }

    // ========================================================================
    // 遥测
    // ========================================================================

    private void updateTelemetry(double rx, double ry, double hdg,
                                  double vx, double vy, double omg,
                                  AutoAimSubsystem_MK2.TurretCommand cmd, double loopMs) {
        joinedTele.addData("alliance", getAllianceName());
        joinedTele.addData("loopMs", loopMs);
        joinedTele.addData("x", rx);
        joinedTele.addData("y", ry);
        joinedTele.addData("h", hdg);
        joinedTele.addData("targetX", getTargetX());
        joinedTele.addData("targetY", getTargetY());
        joinedTele.addData("vx", vx);
        joinedTele.addData("vy", vy);
        joinedTele.addData("omegaDeg", omg);
        joinedTele.addData("shooterOn", shooterOn);
        joinedTele.addData("shootOnTheMove", shootOnTheMove);
        joinedTele.addData("manualDistanceMode", manualDistanceMode);
        joinedTele.addData("autoAimHasTarget", cmd.hasTarget);
        joinedTele.addData("aimLocked", cmd.isAimLocked);
        joinedTele.addData("targetDist", cmd.targetDist);
        joinedTele.addData("flightTime", cmd.flightTime);
        joinedTele.addData("targetTurretAngle", cmd.targetTurretAngle);
        joinedTele.addData("turretDeg(编码器)", autoAim.getCurrentTurretAngle());
        joinedTele.addData("turretDeg(滤波后)", autoAim.getFilteredTurretAngle());
        joinedTele.addData("aimError", cmd.aimError);
        joinedTele.addData("yawCorrection", yawCorrection);
        joinedTele.addData("distanceCorrection", distanceCorrection);
        joinedTele.addData("targetVelocity(rpm)", targetVelocity);
        joinedTele.addData("shooterVL(实际rpm)", robot.shooter.leftShooter.getVelocity());
        joinedTele.addData("shooterVR(实际rpm)", robot.shooter.rightShooter.getVelocity());
        joinedTele.addData("targetPitch(浮板角度)", cmd.targetPitch);
        joinedTele.addData("intakePower", robot.shooter.calculateIntakePower());
        joinedTele.addData("battery(V)", autoAim.getCurrentBatteryVoltage());
        joinedTele.update();
    }

    // ========================================================================
    // 校验
    // ========================================================================

    /** 确保所有手柄映射已赋值，避免 NPE。 */
    private void validateMapping() {
        if (mapping.chassis    == null) mapping.chassis    = gamepad1;
        if (mapping.actions    == null) mapping.actions    = gamepad1;
        if (mapping.toggles    == null) mapping.toggles    = gamepad1;
        if (mapping.trims      == null) mapping.trims      = gamepad1;
        if (mapping.relocalize == null) mapping.relocalize = gamepad1;
    }
}
