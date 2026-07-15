package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.*;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.robotConstants;

/**
 * ============================================================================
 * AutoAimSubsystem —— 完整自瞄管线（重构版）
 * ============================================================================
 *
 * <p><b>架构：一个主类 + 五个内部静态类，全部在一个文件里。</b>
 *
 * <pre>
 *   AutoAimSubsystem          — 编排器（~100 行）
 *     ├── TurretCommand       — 每帧返回的结果 DTO
 *     ├── EmaFilter           — 通用指数滑动平均滤波器
 *     ├── AimCalculator       — 距离 → 转速/浮板 多项式查表（系数与 Shooter.java 一致）
 *     ├── TurretController    — 转塔前馈+PID+电压补偿+刹车预判
 *     └── ShootOnTheMovePredictor — 底盘滤波+朝向融合+移动发射预判
 * </pre>
 *
 * <p>每帧调用一次 {@link #update}，返回 {@link TurretCommand}：
 * <ul>
 *   <li>{@code command.hasTarget} — 是否有有效瞄准解</li>
 *   <li>{@code command.targetRpm} — 飞轮目标转速，调用方自己喂给 Shooter</li>
 *   <li>{@code command.isAimLocked} — 误差已收敛到容差以内</li>
 * </ul>
 * 转塔和浮板舵机由本类内部直接驱动，调用方不需要管。
 */
@Configurable
public class AutoAimSubsystem_MK2 {

    // ========================================================================
    // 内部类 1：结果 DTO
    // ========================================================================

    public static class TurretCommand {
        public boolean hasTarget = false;
        public double targetRpm = 0.0;
        public double targetPitch = 0.0;
        public boolean isAimLocked = false;
        public double currentTolerance = 1.0;
        public double targetDist = 0.0;
        public double targetTurretAngle = 0.0;
        public double aimError = 0.0;
        public double flightTime = 0.0;

        void reset() {
            hasTarget = false; targetRpm = 0.0; targetPitch = 0.0;
            isAimLocked = false; currentTolerance = 1.0;
            targetDist = 0.0; targetTurretAngle = 0.0;
            aimError = 0.0; flightTime = 0.0;
        }
    }

    // ========================================================================
    // 内部类 2：EMA 滤波器
    // ========================================================================

    /** 通用指数滑动平均滤波器，消除散落在各处重复的"值+alpha+初始化标记"三元组。 */
    public static class EmaFilter {
        private double value;
        private boolean init;
        private final double alpha;

        public EmaFilter(double alpha) {
            if (alpha <= 0.0 || alpha > 1.0)
                throw new IllegalArgumentException("alpha must be in (0,1]");
            this.alpha = alpha;
        }

        /** 喂入原始值并返回滤波值。首帧用原始值初始化。 */
        public double update(double raw) {
            if (!init) { value = raw; init = true; }
            else        value += alpha * (raw - value);
            return value;
        }

        public double get()    { return value; }
        public void set(double v) { value = v; init = true; }
        public void reset()   { init = false; value = 0.0; }
        public boolean isInit(){ return init; }
    }

    // ========================================================================
    // 内部类 3：距离 → 转速/浮板 多项式查表
    // ========================================================================
    public static class AimCalculator {

        // ----  原始系数：velocity cubic ----
        public static double RPM_A = robotConstants.RPM_A;
        public static double RPM_B =  robotConstants.RPM_B;
        public static double RPM_C =  robotConstants.RPM_C;
        public static double RPM_D =  robotConstants.RPM_D;

        // ----  原始系数：panel cubic ----
        public static double PANEL_A = robotConstants.PANEL_A;
        public static double PANEL_B = robotConstants.PANEL_B;
        public static double PANEL_C = robotConstants.PANEL_C;
        public static double PANEL_D = robotConstants.PANEL_D;

        // ---- 浮板限位 ----
        public static double PANEL_MIN = robotConstants.PANEL_MIN;
        public static double PANEL_MAX = robotConstants.PANEL_MAX;

        private AimCalculator() {}

        public static double getRpm(double dist) {
            return cubic(RPM_A, RPM_B, RPM_C, RPM_D, dist);
        }

        public static double getPitch(double dist) {
            return Range.clip(cubic(PANEL_A, PANEL_B, PANEL_C, PANEL_D, dist),
                              PANEL_MIN, PANEL_MAX);
        }

        /** 根据机器人位置和目标点，返回完整瞄准解算结果；距离<0.1英寸返回null。 */
        public static AimResult solveAim(double robotX, double robotY,
                                         double targetX, double targetY,
                                         double flightTime, double distanceOffset) {
            double dx = targetX - robotX, dy = targetY - robotY;
            double dist = Math.hypot(dx, dy);
            if (dist < 0.1) return null;
            double shotDist = Math.max(0.0, dist + distanceOffset);
            return new AimResult(dist, Math.toDegrees(Math.atan2(dy, dx)),
                                 getRpm(shotDist), getPitch(shotDist), flightTime);
        }

        private static double cubic(double a, double b, double c, double d, double x) {
            return ((a * x + b) * x + c) * x + d; // Horner 形式，比 a*x³+... 少一次乘法
        }

        public static class AimResult {
            public final double dist, algYaw, rpm, pitch, flightTime;
            public AimResult(double dist, double yaw, double rpm, double pitch, double ft) {
                this.dist = dist; this.algYaw = yaw; this.rpm = rpm;
                this.pitch = pitch; this.flightTime = ft;
            }
        }
    }

    // ========================================================================
    // 内部类 4：转塔控制器（前馈 + PID + 电压补偿 + 刹车预判）
    // ========================================================================

    @Configurable
    public static class TurretController {

        public static double kP = 30.0, kI = 0.0, kD = 0.0, kF = 0.0;
        public static double kV = 0.001339, kS = 0.024370, kA = 0.000007;
        public static double kLinearBraking = 0.069085;
        public static double POWER_SIGN = 1.0, MAX_POWER = 1.0;
        public static double TUNING_VOLTAGE = 13.84;

        public static double MIN_SAFE_ANGLE = -175.0, MAX_SAFE_ANGLE = 175.0;
        public static double CENTER_ANGLE = 0.0;
        public static boolean RESET_ENCODER_ON_INIT = false;

        // ---- 滤波器 ----
        public static double ANGLE_ALPHA = 0.8, VEL_ALPHA = 0.8, ACCEL_ALPHA = 0.15;

        // ---- 硬件 ----
        private DcMotorEx motor;
        private Servo pitchServo;
        private HardwareMap hwMap;

        // ---- 控制状态 ----
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
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pitchServo = hm.get(Servo.class, org.firstinspires.ftc.teamcode.constants.robotConfigs.PANEL);
            battery = readBattery(); pidf.setPIDF(kP, kI, kD, kF);
            setPitch(AimCalculator.PANEL_MIN);
        }

        /**
         * @param curAngle  当前滤波/刹车预判后的转塔相对角度(度)
         * @param tgtAngle  目标角度，内部会夹到安全限位
         * @param ffVel     前馈角速度(度/秒)，如抵消自转或追踪移动目标
         * @param dt        帧间隔(秒)，已夹持[5ms,100ms]
         */
        public void update(double curAngle, double tgtAngle, double ffVel, double dt) {
            dt = clampDt(dt);
            pidf.setPIDF(kP, kI, kD, kF); refVolt();
            updateFilters(dt);

            tgtAngle = Range.clip(tgtAngle, MIN_SAFE_ANGLE, MAX_SAFE_ANGLE);

            double pidVel  = pidf.calculate(curAngle, tgtAngle);
            double tgtVel  = pidVel + ffVel;
            double rawAccel = (tgtVel - lastTargetVel) / dt;
            lastTargetVel = tgtVel;
            double filtAccel = accelF.update(rawAccel);

            double ks = Math.abs(tgtVel) > 0.5 ? Math.signum(tgtVel) : 0.0;
            double pwr = filtAccel * kA + tgtVel * kV + ks * kS;
            pwr *= TUNING_VOLTAGE / battery;
            motor.setPower(Range.clip(POWER_SIGN * pwr, -MAX_POWER, MAX_POWER));
        }

        /** 刹车过冲预判：当前速度与期望修正方向相反时，把惯性冲程折算进当前角度。 */
        public double predictBraking(double curAngle, double error) {
            double v = velF.get();
            double d = kLinearBraking * Math.abs(v);
            if (Math.signum(error) != Math.signum(v) && Math.abs(v) > 0.5)
                return curAngle + Math.signum(v) * d;
            return curAngle;
        }

        public void halt()          { motor.setPower(0.0); }
        public double rawAngle()   { return motor.getCurrentPosition() / (double)TURRET_FULL_RANGE_ENCODER * TURRET_FULL_RANGE_DEGREE; }
        public double filtAngle()  { return angleF.get(); }
        public double filtVel()    { return velF.get(); }
        public double getBattery() { return battery; }

        public void setPitch(double p) { pitchServo.setPosition(Range.clip(p, AimCalculator.PANEL_MIN, AimCalculator.PANEL_MAX)); }

        public void stop() { motor.setPower(0); angleF.reset(); velF.reset(); accelF.reset(); lastTargetVel = 0; }

        // ---- internal ----
        private void updateFilters(double dt) {
            double raw = rawAngle(); angleF.update(raw);
            if (velF.isInit() && dt > 1e-6) velF.update((raw - lastRawAngle) / dt);
            lastRawAngle = raw;
        }
        private void refVolt() {
            long n = System.nanoTime();
            if (n - lastVoltTime > 250_000_000L) { battery = readBattery(); lastVoltTime = n; }
        }
        private double readBattery() { double m=0; for (VoltageSensor s:hwMap.voltageSensor) m=Math.max(m,s.getVoltage()); return Math.max(8,m>0?m:TUNING_VOLTAGE); }
        private static double clampDt(double dt) { if(dt<0.005) return 0.005; if(dt>0.100) return 0.100; return dt; }
    }

    // ========================================================================
    // 内部类 5：移动发射预判器
    // ========================================================================

    @Configurable
    public static class ShootOnTheMovePredictor {

        // ---- 滤波系数 ----
        public static double VEL_ALPHA = 0.3, ACCEL_ALPHA = 0.4;
        public static double OMEGA_ALPHA = 0.7, HEADING_CORR_ALPHA = 0.5;

        // ---- 刹车模型 ----
        public static double BRAKE_WEIGHT = 0.55;
        public static double FWD_BRAKE_DECEL = 33.09567766;
        public static double LAT_BRAKE_DECEL = 52.88478403;

        // ---- 飞行时间模型 ----
        public static double BASE_FT = 0.4, FT_PER_IN = 0.00575;
        public static double MIN_FT = 0.35, MAX_FT = 1.25;
        public static double MECH_DELAY = 0.0;
        public static double SHOT_DIST_OFFSET = 0.0;

        // ---- 状态 ----
        private final EmaFilter fwdF   = new EmaFilter(VEL_ALPHA);
        private final EmaFilter latF   = new EmaFilter(VEL_ALPHA);
        private final EmaFilter fwdAcc = new EmaFilter(ACCEL_ALPHA);
        private final EmaFilter latAcc = new EmaFilter(ACCEL_ALPHA);
        private final EmaFilter omegaF = new EmaFilter(OMEGA_ALPHA);

        private double lastFwd, lastLat;
        private double smoothHdg, filtOmega;
        private boolean hdgInit;

        private boolean enabled = true;

        // ---- 本帧缓存 ----
        private double fwdVel, latVel, fwdPredAcc, latPredAcc;
        private double effVx, effVy;
        private double futureX, futureY, lastFt;
        private double cacheRx = Double.NaN, cacheRy = Double.NaN;

        public void setEnabled(boolean e) { enabled = e; }
        public boolean isEnabled()        { return enabled; }

        /** 每帧调用。处理坐标变换、速度/加速度滤波、朝向融合、刹车感知加速度预测。 */
        public void update(double rx, double ry, double gvx, double gvy,
                           double measHdg, double omegaDeg, boolean braking, double dt) {
            dt = clampDt(dt);
            updateHeading(measHdg, omegaDeg, dt);
            double hr = Math.toRadians(smoothHdg);
            double ch = Math.cos(hr), sh = Math.sin(hr);

            double rfwd = gvx*ch + gvy*sh;
            double rlat = -gvx*sh + gvy*ch;
            fwdVel = fwdF.update(rfwd);
            latVel = latF.update(rlat);

            if (fwdF.isInit() && dt > 1e-6) {
                fwdAcc.update((fwdVel - lastFwd) / dt);
                latAcc.update((latVel - lastLat) / dt);
            }
            lastFwd = fwdVel; lastLat = latVel;

            fwdPredAcc = predAxis(fwdVel, fwdAcc.get(), braking, FWD_BRAKE_DECEL);
            latPredAcc = predAxis(latVel, latAcc.get(), braking, LAT_BRAKE_DECEL);

            effVx = fwdVel*ch - latVel*sh;
            effVy = fwdVel*sh + latVel*ch;

            cacheRx = Double.NaN; // 让 computeFuture 在下次 getFuture 时重新算
        }

        public double getFutureX(double tx, double ty, double rx, double ry) { compute(tx, ty, rx, ry); return futureX; }
        public double getFutureY(double tx, double ty, double rx, double ry) { compute(tx, ty, rx, ry); return futureY; }
        public double getLastFlightTime() { return lastFt; }

        /** 目标方位线因机器人平移而产生的角速度(度/秒)，作为转塔前馈。 */
        public double translationalOmega(double tx, double ty, double rx, double ry) {
            double dx=tx-rx, dy=ty-ry, ds=dx*dx+dy*dy;
            if (ds<0.001) return 0;
            return Math.toDegrees((-dx*effVy + dy*effVx)/ds);
        }

        public double getSmoothHeading() { return smoothHdg; }
        public double getFiltOmega()     { return filtOmega; }
        public double getFwdVel()        { return fwdVel; }
        public double getLatVel()        { return latVel; }

        public void reset() {
            fwdF.reset(); latF.reset(); fwdAcc.reset(); latAcc.reset(); omegaF.reset();
            hdgInit=false; smoothHdg=0; filtOmega=0; lastFwd=0; lastLat=0;
            lastFt=0; futureX=0; futureY=0; cacheRx=Double.NaN;
        }

        // ---- internal ----
        private void compute(double tx, double ty, double rx, double ry) {
            if (rx == cacheRx && ry == cacheRy) return;
            cacheRx = rx; cacheRy = ry;
            if (!enabled) { futureX = rx; futureY = ry; lastFt = 0; return; }

            double dist = Math.hypot(tx-rx, ty-ry);
            lastFt = Range.clip(BASE_FT + FT_PER_IN * dist, MIN_FT, MAX_FT);

            double hr = Math.toRadians(smoothHdg);
            double ch = Math.cos(hr), sh = Math.sin(hr);

            double relFwd = fwdVel + fwdPredAcc * MECH_DELAY;
            double relLat = latVel + latPredAcc * MECH_DELAY;
            relFwd = stopAtZero(fwdVel, relFwd);
            relLat = stopAtZero(latVel, relLat);

            double af = 0.5*(fwdVel+relFwd), al = 0.5*(latVel+relLat);
            double rx2 = rx + (af*ch - al*sh) * MECH_DELAY;
            double ry2 = ry + (af*sh + al*ch) * MECH_DELAY;

            double rvx = relFwd*ch - relLat*sh;
            double rvy = relFwd*sh + relLat*ch;
            futureX = rx2 + rvx * lastFt;
            futureY = ry2 + rvy * lastFt;
        }

        private void updateHeading(double meas, double omega, double dt) {
            if (!hdgInit) { smoothHdg = meas; filtOmega = omega; hdgInit = true; return; }
            filtOmega += OMEGA_ALPHA * (omega - filtOmega);
            smoothHdg += filtOmega * dt;
            double e = normDeg(meas - smoothHdg);
            smoothHdg = normDeg(smoothHdg + HEADING_CORR_ALPHA * e);
        }

        private double predAxis(double v, double aMeas, boolean braking, double decel) {
            if (!braking || Math.abs(v) <= 0.0001) return aMeas;
            double aBrake = -Math.signum(v) * decel;
            return aMeas * (1-BRAKE_WEIGHT) + aBrake * BRAKE_WEIGHT;
        }

        private static double stopAtZero(double cur, double pred) {
            if (cur>0&&pred<0 || cur<0&&pred>0) return 0; return pred;
        }
        private static double normDeg(double a) { while(a>180)a-=360; while(a<-180)a-= -360; return a; }
        private static double clampDt(double dt) { if(dt<0.005)return 0.005; if(dt>0.100)return 0.100; return dt; }
    }

    // ========================================================================
    // AutoAimSubsystem 自身（编排器）
    // ========================================================================

    // ---- 可调参数 ----
    public static double TOL_NEAR_DEG = 30, TOL_FAR_DEG = 5;
    public static double TOL_NEAR_IN  = 20, TOL_FAR_IN  = 90;
    public static double TURRET_LATENCY = 0.012;
    public static double SHOT_DIST_OFFSET = 0.0;

    // ---- 组件 ----
    private final TurretController        turret    = new TurretController();
    private final ShootOnTheMovePredictor predictor = new ShootOnTheMovePredictor();
    private final TurretCommand           cmd       = new TurretCommand();

    private long lastNanos;

    // ======================== 初始化 ========================

    public AutoAimSubsystem_MK2() {}
    public AutoAimSubsystem_MK2(HardwareMap hm) { init(hm); }

    public void init(HardwareMap hm) { turret.init(hm); }

    // ======================== 每帧主入口 ========================

    /**
     * @param rx,ry          机器人场地坐标(英寸)
     * @param gvx,gvy        场地坐标系速度(in/s)
     * @param measHdg        原始朝向(度)
     * @param omega          角速度(度/s)
     * @param tx,ty          目标点(球门)场地坐标
     * @param manualMode     true→用 manualDist 固定距离，不走里程计
     * @param manualDist     手动模式的距离值
     * @param shootOnMove    是否启用移动发射预判
     * @param braking        摇杆是否回中(刹车中)
     * @param yawOff         操作手方向键 yaw 微调(度)
     * @return 本帧瞄准结果(复用实例，只在本帧内有效)
     */
    public TurretCommand update(
            double rx, double ry, double gvx, double gvy,
            double measHdg, double omega,
            double tx, double ty,
            boolean manualMode, double manualDist,
            boolean shootOnMove, boolean braking, double yawOff) {

        cmd.reset();
        double dt = elapsed();

        // 1. 底盘估计 + 移动预判
        predictor.setEnabled(shootOnMove);
        predictor.update(rx, ry, gvx, gvy, measHdg, omega, braking, dt);

        double smoothH = predictor.getSmoothHeading();

        // 2. 解算瞄准点
        AimCalculator.AimResult aim;
        if (manualMode) {
            aim = new AimCalculator.AimResult(manualDist, smoothH,
                    AimCalculator.getRpm(manualDist),
                    AimCalculator.getPitch(manualDist), 0.0);
        } else {
            double fx = predictor.getFutureX(tx, ty, rx, ry);
            double fy = predictor.getFutureY(tx, ty, rx, ry);
            aim = AimCalculator.solveAim(fx, fy, tx, ty,
                    predictor.getLastFlightTime(), SHOT_DIST_OFFSET);
        }

        if (aim == null) { turret.halt(); return cmd; }

        // 3. 填充结果
        cmd.hasTarget = true;
        cmd.targetRpm = aim.rpm;
        cmd.targetPitch = aim.pitch;
        cmd.targetDist = aim.dist;
        cmd.flightTime = aim.flightTime;
        cmd.currentTolerance = tolerance(aim.dist);

        // 4. 方位角补偿（移动目标 + 手动微调）
        double transOmega = predictor.translationalOmega(tx, ty, rx, ry);
        double absTarget = aim.algYaw + yawOff + transOmega * TURRET_LATENCY;

        // 5. 转塔相对角度
        double curAbs = smoothH + turret.filtAngle();
        double error  = normDeg(absTarget - curAbs);
        double relTgt = turret.filtAngle() + error;
        cmd.targetTurretAngle = relTgt;
        cmd.aimError = error;

        // 6. 安全限位
        if (relTgt < TurretController.MIN_SAFE_ANGLE || relTgt > TurretController.MAX_SAFE_ANGLE) {
            cmd.hasTarget = false; cmd.isAimLocked = false;
            turret.update(turret.filtAngle(), TurretController.CENTER_ANGLE,
                    -predictor.getFiltOmega(), dt);
            turret.setPitch(AimCalculator.PANEL_MIN);
            return cmd;
        }

        cmd.isAimLocked = Math.abs(error) <= cmd.currentTolerance;

        // 7. 刹车过冲预判
        double drvAngle = turret.predictBraking(turret.filtAngle(), error);

        // 8. 驱动转塔 + 浮板
        turret.update(drvAngle, relTgt, -predictor.getFiltOmega() + transOmega, dt);
        turret.setPitch(aim.pitch);

        return cmd;
    }

    // ======================== 遥测 ========================

    public double getCurrentTurretAngle()    { return turret.rawAngle(); }
    public double getFilteredTurretAngle()   { return turret.filtAngle(); }
    public double getFilteredTurretVelocity(){ return turret.filtVel(); }
    public double getCurrentBatteryVoltage() { return turret.getBattery(); }
    public double getLastTargetTurretAngle() { return cmd.targetTurretAngle; }
    public double getLastAimError()          { return cmd.aimError; }
    public double getLastFlightTime()        { return predictor.getLastFlightTime(); }

    /** @deprecated use {@link #getLastTargetTurretAngle()} */
    @Deprecated public double getLastTargetAbsAngle() { return cmd.targetTurretAngle; }

    public void stop() { turret.stop(); predictor.reset(); }

    // ---- internal ----
    private double elapsed() {
        long n = System.nanoTime();
        double dt = lastNanos == 0 ? 0.0001 : (n - lastNanos) / 1e9;
        lastNanos = n;
        if (dt < 0.005) dt = 0.005;
        if (dt > 0.100) dt = 0.100;
        return dt;
    }

    private static double tolerance(double d) {
        if (d <= TOL_NEAR_IN) return TOL_NEAR_DEG;
        if (d >= TOL_FAR_IN)  return TOL_FAR_DEG;
        return TOL_NEAR_DEG + (TOL_FAR_DEG - TOL_NEAR_DEG) * (d - TOL_NEAR_IN) / (TOL_FAR_IN - TOL_NEAR_IN);
    }

    private static double normDeg(double a) { while(a>180)a-=360; while(a<-180)a-= -360; return a; }
}
