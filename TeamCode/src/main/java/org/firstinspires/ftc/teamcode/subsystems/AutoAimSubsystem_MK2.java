package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.robotConstants.*;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * ============================================================================
 * AutoAimSubsystem_MK2 —— 精简版：只负责"转塔转到正确角度"，且比电机固件自带的
 * RUN_TO_POSITION 更快。
 * ============================================================================
 *
 * <h3>这次为什么精简成这样</h3>
 * 参照你们已经测试稳定的 {@code A_2_AA_AS_Red/Blue.java}：飞轮转速/浮板角度全部由
 * {@code Shooter.setShooterByDis()/setShooterByDisShow()} 处理（系数统一从
 * {@code robotConstants.RPM_A/B/C/D}、{@code PANEL_A/B/C/D} 读取，跟这里完全一致，
 * 不需要在这个类里再维护一份重复的插值逻辑）；移动发射的提前量预判也是 A_2 那套
 * 简单公式，直接写在 TeleOp 里，不需要这个类操心底盘滤波/飞行时间。
 * 这个类现在**只做一件事**：给坐标+朝向+目标点，把转塔转到该转的角度，比
 * {@code Shooter.turretToDegree()}(电机固件RUN_TO_POSITION，稳但慢) 更快。
 *
 * <h3>安全限位</h3>
 * 沿用 A_2 里 {@code TURRET_ABS_RANGE_DEGREE}(=170) 这个阈值，逻辑跟 A_2 一致：
 * 目标角度超出 ±TURRET_ABS_RANGE_DEGREE 就放弃瞄准、转回中心——但这里**修了 A_2 里
 * 那个没做归一化的老bug**（A_2 直接拿 {@code targetATAN - drivetrainHeading} 比大小，
 * 两个角度差可能算出300+度这种没意义的值；这里先 {@code normalizeDeg} 到最短夹角
 * 再比较）。
 *
 * <h3>调用方式</h3>
 * <pre>{@code
 * autoAim.init(hardwareMap);          // waitForStart() 之前调一次
 * ...
 * AutoAimSubsystem_MK2.TurretCommand cmd = autoAim.update(
 *         robotX, robotY, headingDeg, omegaDeg, targetX, targetY, yawOffset);
 * // cmd.hasTarget / cmd.targetTurretAngle / cmd.aimError 仅供参考/打遥测，
 * // 转塔已经在 update() 内部被直接驱动了，不需要调用方再做任何事
 * ...
 * autoAim.stop();                     // OpMode 结束前调一次
 * }</pre>
 */
@Configurable
public class AutoAimSubsystem_MK2 {

    /** 每帧调用结果，仅供打遥测参考，转塔已经在 update() 内部被驱动完了。 */
    public static class TurretCommand {
        public boolean hasTarget = false;
        public double targetDist = 0.0;
        public double targetTurretAngle = 0.0; // 转塔目标相对角度(度)
        public double aimError = 0.0;          // 归一化后的角度误差(度)

        void reset() {
            hasTarget = false;
            targetDist = 0.0;
            targetTurretAngle = 0.0;
            aimError = 0.0;
        }
    }

    /** 通用指数滑动平均滤波器。 */
    public static class EmaFilter {
        private double value;
        private boolean init;
        private final double alpha;

        public EmaFilter(double alpha) {
            if (alpha <= 0.0 || alpha > 1.0) throw new IllegalArgumentException("alpha must be in (0,1]");
            this.alpha = alpha;
        }

        public double update(double raw) {
            if (!init) { value = raw; init = true; }
            else        value += alpha * (raw - value);
            return value;
        }

        public double get() { return value; }
        public void reset() { init = false; value = 0.0; }
        public boolean isInit() { return init; }
    }

    /**
     * 转塔控制器：前馈(kS/kV/kA) + PID位置环 + 电压补偿 + 刹车过冲预判。
     * 只碰转塔电机，不碰浮板(浮板归 Shooter 管)。
     */
    @Configurable
    public static class TurretController {

        public static double kP = 30.0, kI = 0.0, kD = 0.0, kF = 0.0;
        public static double kV = 0.001339, kS = 0.024370, kA = 0.000007;
        public static double kLinearBraking = 0.069085;
        public static double POWER_SIGN = 1.0, MAX_POWER = 1.0;
        public static double TUNING_VOLTAGE = 13.84;

        // 默认按 robotConstants.TURRET_ABS_RANGE_DEGREE(=170) 对称设置，
        // 如果实测左右两边到线材极限不对称，改这两个值即可。
        public static double MIN_SAFE_ANGLE = -TURRET_ABS_RANGE_DEGREE;
        public static double MAX_SAFE_ANGLE = TURRET_ABS_RANGE_DEGREE;
        public static double CENTER_ANGLE = 0.0;
        public static boolean RESET_ENCODER_ON_INIT = false;

        public static double ANGLE_ALPHA = 0.8, VEL_ALPHA = 0.8, ACCEL_ALPHA = 0.15;

        private DcMotorEx motor;
        private HardwareMap hwMap;

        private final PIDFController pidf = new PIDFController(kP, kI, kD, kF);
        private final EmaFilter angleF = new EmaFilter(ANGLE_ALPHA);
        private final EmaFilter velF   = new EmaFilter(VEL_ALPHA);
        private final EmaFilter accelF = new EmaFilter(ACCEL_ALPHA);

        private double lastRawAngle, lastTargetVel;
        private long lastVoltTime;
        private double battery = 12.0;

        public void init(HardwareMap hm) {
            hwMap = hm;
            motor = hm.get(DcMotorEx.class, org.firstinspires.ftc.teamcode.constants.robotConfigs.TURRET);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            if (RESET_ENCODER_ON_INIT) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // 用 RUN_WITHOUT_ENCODER 而不是电机固件的 RUN_TO_POSITION——
            // 这就是比 Shooter.turretToDegree() 快的关键，怎么转/怎么减速全靠下面自己算
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            battery = readBattery();
            pidf.setPIDF(kP, kI, kD, kF);
        }

        /**
         * @param curAngle 当前(或刹车预判修正后)的转塔相对角度(度)
         * @param tgtAngle 目标角度，内部会夹到安全限位
         * @param ffVel    前馈角速度(度/秒)，比如抵消机器人自转(-机器人角速度)
         * @param dt       帧间隔(秒)，内部会夹持在[5ms,100ms]防止异常值
         */
        public void update(double curAngle, double tgtAngle, double ffVel, double dt) {
            dt = clampDt(dt);
            pidf.setPIDF(kP, kI, kD, kF); // 每帧重设，方便FTC Dashboard实时调参立刻生效
            refVolt();
            updateFilters(dt);

            tgtAngle = Range.clip(tgtAngle, MIN_SAFE_ANGLE, MAX_SAFE_ANGLE);

            double pidVel   = pidf.calculate(curAngle, tgtAngle);
            double tgtVel   = pidVel + ffVel;
            double rawAccel = (tgtVel - lastTargetVel) / dt;
            lastTargetVel = tgtVel;
            double filtAccel = accelF.update(rawAccel);

            // kS只在真的需要动(期望速度>0.5度/秒)时才加，避免静止时因噪声正负跳变而抖动
            double ks = Math.abs(tgtVel) > 0.5 ? Math.signum(tgtVel) : 0.0;
            double pwr = filtAccel * kA + tgtVel * kV + ks * kS;
            pwr *= TUNING_VOLTAGE / battery; // 电压补偿
            motor.setPower(Range.clip(POWER_SIGN * pwr, -MAX_POWER, MAX_POWER));
        }

        /** 刹车过冲预判：转塔正在冲、但目标要求反向修正时，把惯性冲程折算进当前角度，让PID提前反应。 */
        public double predictBraking(double curAngle, double error) {
            double v = velF.get();
            double d = kLinearBraking * Math.abs(v);
            if (Math.signum(error) != Math.signum(v) && Math.abs(v) > 0.5) {
                return curAngle + Math.signum(v) * d;
            }
            return curAngle;
        }

        public void halt() { motor.setPower(0.0); }
        public double rawAngle() { return motor.getCurrentPosition() / (double) TURRET_FULL_RANGE_ENCODER * TURRET_FULL_RANGE_DEGREE; }
        public double filtAngle() { return angleF.get(); }
        public double filtVel()   { return velF.get(); }
        public double getBattery(){ return battery; }

        public void stop() {
            motor.setPower(0);
            angleF.reset(); velF.reset(); accelF.reset();
            lastTargetVel = 0;
        }

        private void updateFilters(double dt) {
            double raw = rawAngle();
            angleF.update(raw);
            if (velF.isInit() && dt > 1e-6) velF.update((raw - lastRawAngle) / dt);
            lastRawAngle = raw;
        }

        private void refVolt() {
            long n = System.nanoTime();
            if (n - lastVoltTime > 250_000_000L) { battery = readBattery(); lastVoltTime = n; }
        }

        private double readBattery() {
            double m = 0;
            for (VoltageSensor s : hwMap.voltageSensor) m = Math.max(m, s.getVoltage());
            return Math.max(8, m > 0 ? m : TUNING_VOLTAGE);
        }

        private static double clampDt(double dt) {
            if (dt < 0.005) return 0.005;
            if (dt > 0.100) return 0.100;
            return dt;
        }
    }

    // ========================================================================
    // 编排器本体
    // ========================================================================

    private final TurretController turret = new TurretController();
    private final TurretCommand cmd = new TurretCommand();
    private long lastTime = 0;

    public void init(HardwareMap hm) {
        turret.init(hm);
    }

    /**
     * 每帧调用一次，内部直接把转塔驱动到该转的角度。
     *
     * @param robotX, robotY   机器人当前场地坐标(英寸)
     * @param headingDeg       机器人当前朝向(度)
     * @param omegaDeg         机器人当前角速度(度/秒)
     * @param targetX, targetY 要瞄准的目标点场地坐标(英寸)——如果启用了移动发射提前量，
     *                         调用方(TeleOp)自己按A_2的公式把提前量加进这两个坐标里再传进来
     * @param yawOffset        操作手方向键手动微调(度)
     */
    public TurretCommand update(double robotX, double robotY, double headingDeg, double omegaDeg,
                                double targetX, double targetY, double yawOffset) {
        cmd.reset();

        long now = System.nanoTime();
        double dt = lastTime == 0 ? 0.0001 : (now - lastTime) / 1e9;
        lastTime = now;

        double dx = targetX - robotX, dy = targetY - robotY;
        double dist = Math.hypot(dx, dy);
        if (dist < 0.1) {
            turret.halt();
            return cmd;
        }
        cmd.hasTarget = true;
        cmd.targetDist = dist;

        double absTarget = Math.toDegrees(Math.atan2(dy, dx)) + yawOffset;
        double error = normalizeDeg(absTarget - headingDeg - turret.filtAngle());
        double relTarget = turret.filtAngle() + error;

        cmd.targetTurretAngle = relTarget;
        cmd.aimError = error;

        if (relTarget < TurretController.MIN_SAFE_ANGLE || relTarget > TurretController.MAX_SAFE_ANGLE) {
            // 超出安全限位：跟A_2一样，放弃瞄准，转回中心
            cmd.hasTarget = false;
            cmd.targetTurretAngle = TurretController.CENTER_ANGLE;
            turret.update(turret.filtAngle(), TurretController.CENTER_ANGLE, -omegaDeg, dt);
            return cmd;
        }

        double predictedAngle = turret.predictBraking(turret.filtAngle(), error);
        turret.update(predictedAngle, relTarget, -omegaDeg, dt);
        return cmd;
    }

    public double getCurrentTurretAngle()  { return turret.rawAngle(); }
    public double getFilteredTurretAngle() { return turret.filtAngle(); }
    public double getFilteredTurretVelocity() { return turret.filtVel(); }
    public double getCurrentBatteryVoltage()  { return turret.getBattery(); }

    /**
     * 转塔自瞄关闭时调用：转塔转回中心(0度)锁定，不瞄准任何目标。
     * 跟 update() 用同一套滤波+前馈+PID，只是目标角度固定为 CENTER_ANGLE——
     * 这样转塔角度/速度滤波器一直保持"热"的，重新打开自瞄时不会有猛转追赶的问题
     * (这个精简版引擎本身没有维护"机器人朝向"的持续状态，所以不存在旧版那种
     * smoothHeading失联的顾虑，center()可以随时安全调用/随时切回update())。
     */
    public TurretCommand center(double omegaDeg) {
        cmd.reset();

        long now = System.nanoTime();
        double dt = lastTime == 0 ? 0.0001 : (now - lastTime) / 1e9;
        lastTime = now;

        turret.update(turret.filtAngle(), TurretController.CENTER_ANGLE, -omegaDeg, dt);

        cmd.hasTarget = false;
        cmd.targetTurretAngle = TurretController.CENTER_ANGLE;
        return cmd;
    }

    public void stop() {
        turret.stop();
        lastTime = 0;
    }

    /** 把任意角度差归一化到 (-180, 180] 区间，保证走的是最短路径夹角——修A_2里没做这一步的老bug。 */
    private static double normalizeDeg(double a) {
        while (a > 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    }
}