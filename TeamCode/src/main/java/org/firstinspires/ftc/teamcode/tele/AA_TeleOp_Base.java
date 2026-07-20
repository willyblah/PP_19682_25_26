package org.firstinspires.ftc.teamcode.tele;

import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndH;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndY;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.targetVelocity;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.AutoAimSubsystem_MK2;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * ============================================================================
 * AA_TeleOp_Base —— 逻辑/键位严格对齐已测试稳定的 A_2_AA_AS_Red/Blue.java
 * ============================================================================
 *
 * <p>跟 A_2 唯一的实质区别：转塔驱动从 {@code Shooter.turretToDegree()}（电机固件自带
 * RUN_TO_POSITION，稳但转得慢）换成了 {@code AutoAimSubsystem_MK2}（自己写的前馈+PID，
 * 更快）。其余——移动发射提前量公式、飞轮/浮板走 Shooter、按键分配、手柄1/手柄2分工——
 * 全部照抄 A_2，没有引入新的复杂功能（之前加的 Pedro Pathing 一键自动跑打宏、多手柄
 * 灵活映射(GamepadRole) 这些这次全部去掉了，只留红/蓝两边共享逻辑这一层抽象）。
 *
 * <h3>手柄键位（跟 A_2 完全一致）</h3>
 * <pre>
 *   手柄1: 摇杆开车 / 右扳机收球(同时强制关闭shooterOn) / 左扳机吐球 /
 *          右保险杠喂球 / 左保险杠切换shooterOn(飞轮+挡球板) / Y切换跑打 / Start复位
 *   手柄2: 方向键上/下 = 距离修正 ±1，方向键左/右 = 转塔角度修正 ∓1
 * </pre>
 *
 * <h3>转塔全程自瞄</h3>
 * 跟 A_2 一样，转塔驱动语句在 shooterOn 的 if/else 分支之外，每帧无条件执行——
 * 不管开没开炮塔，转塔都会一直转向目标。
 *
 * <h3>转塔自瞄独立开关(新增)</h3>
 * X键切换 {@code turretAimEnabled}，跟 {@code shooterOn}(左保险杠，管飞轮) 彻底独立：
 * 开 → 正常自瞄；关 → 转塔锁回0度中心，不再瞄准。这是给"PP(里程计)突然给出异常位姿"
 * 准备的安全阀——一旦发现转塔因为位姿跳变乱转，操作手可以立刻按X把转塔锁住，不影响
 * 飞轮/挡球板/底盘的正常操作。默认开启(跟之前"全程自瞄"的默认行为一致)。
 *
 * <h3>安全限位 bug 修复</h3>
 * A_2 里 {@code Math.abs(targetATAN - drivetrainHeading) <= TURRET_ABS_RANGE_DEGREE}
 * 这一步没有做角度归一化，两个角度相减可能得到300+度这种没意义的值——
 * {@code AutoAimSubsystem_MK2} 内部已经用 normalizeDeg 修好了，行为上更接近你原本想要的
 * 效果（真正的"最短夹角超过170度才放弃瞄准"，而不是被没归一化的角度差误判）。
 */
@Configurable
public abstract class AA_TeleOp_Base extends LinearOpMode {

    // ========================================================================
    // @Configurable 可调参数
    // ========================================================================
    public static double drivePower = 1.0;
    public static double turretCorrection = 0;     // 手柄2方向键左右，跟A_2的int字段同名同用途
    public static double distanceCorrection = 2;   // 手柄2方向键上下，初始值2跟A_2一致

    protected final Robot robot = new Robot();
    protected final AutoAimSubsystem_MK2 autoAim = new AutoAimSubsystem_MK2();
    protected JoinedTelemetry joinedTele;

    // ========================================================================
    // 运行时状态，跟A_2字段一一对应
    // ========================================================================
    private double targetX, targetY, vx, vy;
    private double drivetrainHeading;
    private boolean shooterOn = false;
    private boolean movingShoot = false;
    private boolean turretAimEnabled = true; // X键切换：转塔自瞄总开关，跟shooterOn(飞轮)彻底独立，
    // 万一PP(里程计)突然抽风给出错误位姿，按X能立刻让转塔停止乱转、锁回0位
    private double distance;
    private double at = 0.64; // 移动发射提前量(秒)，公式跟A_2一致

    // ========================================================================
    // 子类必须提供：只有这几项跟颜色相关
    // ========================================================================
    protected abstract double getTargetX();
    protected abstract double getTargetY();
    protected abstract Pose2D getRelocalizePose();
    protected abstract String getAllianceName();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        autoAim.init(hardwareMap);

        resetToAutoEndPose();

        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        joinedTele.addData("alliance", getAllianceName());
        joinedTele.update();

        waitForStart();
        resetToAutoEndPose();

        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, drivePower);

            // ---- 进球/吐球/喂球，跟A_2一致 ----
            if (gamepad1.right_trigger > 0.1) {
                robot.intake.intakeIn();
                shooterOn = false;
            } else if (gamepad1.left_trigger > 0.1) {
                robot.intake.intakeOut(gamepad1.left_trigger);
            } else if (gamepad1.right_bumper) {
                robot.intake.intakeIn(robot.shooter.calculateIntakePower());
            } else {
                robot.intake.intakeStop();
            }

            Pose2D current = robot.drivetrain.getPosition();
            drivetrainHeading = current.getHeading(AngleUnit.DEGREES);
            double robotX = current.getX(DistanceUnit.INCH);
            double robotY = current.getY(DistanceUnit.INCH);
            vx = robot.drivetrain.pinPoint.getVelX(DistanceUnit.INCH);
            vy = robot.drivetrain.pinPoint.getVelY(DistanceUnit.INCH);
            double omegaDeg = robot.drivetrain.pinPoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

            // ---- 移动发射提前量：公式跟A_2完全一致 ----
            double baseTargetX = getTargetX();
            double baseTargetY = getTargetY();
            at = Math.abs(Math.hypot(baseTargetY - robotY, baseTargetX - robotX)) * 0.00575 + 0.4;
            if (movingShoot) {
                targetX = baseTargetX - at * vx;
                targetY = baseTargetY - at * vy;
            } else {
                targetX = baseTargetX;
                targetY = baseTargetY;
            }
            distance = Math.hypot(targetY - robotY, targetX - robotX);

            // ---- 手柄2方向键微调，跟A_2一致（注意符号：左+1/右-1，跟之前版本相反，照抄A_2） ----
            if (gamepad2.dpadUpWasPressed()) distanceCorrection += 1;
            if (gamepad2.dpadDownWasPressed()) distanceCorrection -= 1;
            if (gamepad2.dpadLeftWasPressed()) turretCorrection += 1;
            if (gamepad2.dpadRightWasPressed()) turretCorrection -= 1;
            if (gamepad1.yWasPressed()) movingShoot = !movingShoot;

            if (gamepad1.leftBumperWasPressed()) {
                shooterOn = !shooterOn;
            }

            if (gamepad1.xWasPressed()) {
                turretAimEnabled = !turretAimEnabled;
            }

            if (gamepad1.startWasPressed()) {
                robot.drivetrain.pinPoint.setPosition(getRelocalizePose());
            }

            // ---- 飞轮转速+浮板：跟A_2一致，只有shooterOn=true才真正设定 ----
            if (shooterOn) {
                robot.intake.gateOpen();
                robot.shooter.setShooterByDis(distance + distanceCorrection);
            } else {
                robot.intake.gateClose();
                robot.shooter.shooterHold();
            }
            // shooterOn=false时也顺手算一遍(不真正下发)，让targetVelocity/targetPanel保持新鲜，
            // 这样calculateIntakePower()和遥测在炮塔关闭时也是准的——跟A_2的setShooterByDisShow()用法一致
            robot.shooter.setShooterByDisShow(distance + distanceCorrection);

            // ---- 转塔自瞄：由X键单独控制，跟shooterOn(飞轮)彻底独立 ----
            // 开：正常自瞄，比Shooter.turretToDegree()快；关：转塔锁回0度中心，
            // 用于PP(里程计)突然给出异常位姿时，操作手可以立刻按X让转塔停止乱转
            AutoAimSubsystem_MK2.TurretCommand cmd = turretAimEnabled
                    ? autoAim.update(robotX, robotY, drivetrainHeading, omegaDeg, targetX, targetY, turretCorrection)
                    : autoAim.center(0);

            updateTelemetry(robotX, robotY, cmd);
        }

        autoAim.stop();
        robot.shooter.shooterStop();
        robot.intake.intakeStop();
    }

    private void resetToAutoEndPose() {
        robot.drivetrain.pinPoint.setPosition(new Pose2D(
                DistanceUnit.INCH, autoEndY, 144.0 - autoEndX,
                AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
    }

    private void updateTelemetry(double robotX, double robotY, AutoAimSubsystem_MK2.TurretCommand cmd) {
        joinedTele.addData("alliance", getAllianceName());
        joinedTele.addData("x", robotX);
        joinedTele.addData("y", robotY);
        joinedTele.addData("h", drivetrainHeading);
        joinedTele.addData("targetX", targetX);
        joinedTele.addData("targetY", targetY);
        joinedTele.addData("movingShoot", movingShoot);
        joinedTele.addData("shooterOn", shooterOn);
        joinedTele.addData("turretAimEnabled(X切换)", turretAimEnabled);
        joinedTele.addData("distance", distance);
        joinedTele.addData("distanceCorrection", distanceCorrection);
        joinedTele.addData("turretCorrection", turretCorrection);
        joinedTele.addData("autoAimHasTarget", cmd.hasTarget);
        joinedTele.addData("targetTurretAngle", cmd.targetTurretAngle);
        joinedTele.addData("turretDeg(编码器)", autoAim.getCurrentTurretAngle());
        joinedTele.addData("turretDeg(滤波后)", autoAim.getFilteredTurretAngle());
        joinedTele.addData("turretVel(滤波后)", autoAim.getFilteredTurretVelocity());
        joinedTele.addData("aimError", cmd.aimError);
        joinedTele.addData("shooterT", targetVelocity);
        joinedTele.addData("shooterVL", robot.shooter.leftShooter.getVelocity());
        joinedTele.addData("shooterVR", robot.shooter.rightShooter.getVelocity());
        joinedTele.addData("intakePower", robot.shooter.calculateIntakePower());
        joinedTele.addData("panel", robot.shooter.panel.getPosition());
        joinedTele.addData("battery(V)", autoAim.getCurrentBatteryVoltage());
        joinedTele.update();
    }
}