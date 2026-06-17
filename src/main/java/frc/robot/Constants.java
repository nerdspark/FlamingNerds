package frc.robot;

public class Constants {
    public static final double kp = 5;
    public static final double ki = 0;
    public static final double kd = 0;

    public final class TractionConstants {


        /**
         * WHAT:  The slip ratio the traction controller targets.
         *        Slip ratio = (wheelSpeed - chassisSpeed) / wheelSpeed.
         *        0.0 = perfect grip, 1.0 = full spin.
         *
         * WHY:   Rubber-on-carpet has a "friction peak" at roughly 10–15% slip.
         *        Targeting exactly zero slip leaves grip on the table. Targeting
         *        too high causes sustained wheelspin.
         *
         * TUNE:  Start at 0.10. If the bot still spins out on hard starts, lower
         *        it. If acceleration feels sluggish, nudge up to 0.13–0.15.
         *        Watch wheel velocity vs. chassis velocity in Signal Logger.
         */
        public static final double TARGET_SLIP_RATIO = 0.15;

        /**
         * WHAT:  Slip ratio must exceed this before the controller activates.
         *
         * WHY:   Odometry + encoder noise can produce ~2–4% apparent slip even
         *        when there's none. This dead-band prevents the controller from
         *        fighting noise during normal driving.
         *
         * TUNE:  Should be slightly below TARGET_SLIP_RATIO. If the controller
         *        activates during straight smooth driving, increase this.
         */
        public static final double SLIP_THRESHOLD = 0.06;

        /**
         * WHAT:  Proportional gain for the slip PI controller (Amps / slip-ratio).
         *
         * WHY:   Converts excess slip into a current reduction. Higher = faster
         *        correction. Too high = oscillating torque / wheel chatter.
         *
         * TUNE:  Start at 40. Increase until you see current oscillations in
         *        Signal Logger (wheel velocity bouncing), then back off 20%.
         */
        public static final double SLIP_KP = 40.0;

        /**
         * WHAT:  Integral gain (Amps / (slip-ratio * second)).
         *
         * WHY:   Eliminates steady-state slip on genuinely slick surfaces (wet
         *        carpet, polished tiles). Keep small — too high causes windup.
         *
         * TUNE:  Start at 5.0. Only increase if you see persistent slip error
         *        that KP alone isn't resolving.
         */
        public static final double SLIP_KI = 5.0;

        /**
         * WHAT:  Maximum stator current the traction controller will ever command.
         *
         * WHY:   Hard safety ceiling. The Kraken X60 can handle ~120A peak, but
         *        carpet grip gives out well before that. Also protects the
         *        drivetrain gearbox at high torque.
         *
         * TUNE:  Start at 60A. Increase in 5A steps until wheels spin on hard
         *        starts, then back off one step. Typical FRC swerve: 60–80A.
         */
        public static final double MAX_DRIVE_STATOR_CURRENT = 70.0; // Amps

        /**
         * WHAT:  Clamp on the integral accumulator (Amps).
         *
         * WHY:   Anti-windup. Prevents the integrator from accumulating a huge
         *        correction during sustained spin (e.g., wheels lifted off ground)
         *        that then dumps all at once when grip returns.
         *
         * TUNE:  Set to roughly 15–20% of MAX_DRIVE_STATOR_CURRENT.
         */
        public static final double SLIP_I_MAX_AMPS = 0.2 * MAX_DRIVE_STATOR_CURRENT;

        // ── Launch Control ────────────────────────────────────────────────────

        /**
         * WHAT:  Stator current held during the pre-load phase (robot held
         *        against the wall before launch).
         *
         * WHY:   The Kraken X60's FOC loop requires stator flux to be established
         *        before torque can be produced. Pre-loading at ~20A saturates the
         *        magnetic flux so there's near-zero torque lag at launch.
         *        Also pre-tensions the drivetrain geartrain (removes backlash).
         *
         * TUNE:  20–25A is typically enough for flux saturation. Going higher
         *        just adds heat with no benefit.
         */
        public static final double LAUNCH_PRELOAD_CURRENT = 25.0; // Amps

        /**
         * WHAT:  Peak current commanded at full launch ramp.
         *
         * WHY:   This is the maximum force you want at wheels during the launch
         *        transient. The traction controller will further limit this if
         *        a module slips, so you can be aggressive here.
         *
         * TUNE:  Start at 65A. Increase until traction controller is consistently
         *        limiting (good — means you're at grip limit), or until you tip.
         */
        public static final double LAUNCH_PEAK_CURRENT = 65.0; // Amps

        /**
         * WHAT:  How fast current ramps from pre-load to peak (Amps per second).
         *
         * WHY:   Too fast = all four wheels spin before grip establishes.
         *        Too slow = slow launch, no better than normal driving.
         *        This is the primary launch "feel" tunable.
         *
         * TUNE:  Start at 180 A/s. If you get wheelspin on launch (even with
         *        traction control), slow it down. If launch feels slow, speed up.
         */
        public static final double LAUNCH_RAMP_RATE_APS = 180.0;

        /**
         * WHAT:  Chassis speed (m/s) at which launch control hands off back to
         *        normal driver input.
         *
         * WHY:   Past a certain speed, the robot is already in the "normal"
         *        driving regime — sustained max current is fine and driver
         *        control is more useful than a fixed ramp.
         *
         * TUNE:  ~30–40% of max chassis speed. For a 4–5 m/s bot, use 1.5–2.0.
         */
        public static final double LAUNCH_HANDOFF_SPEED_MPS = 0.1;

        /**
         * WHAT:  Speed range over which authority blends from launch → driver.
         *
         * WHY:   An abrupt handoff causes a noticeable lurch. Blending over
         *        ~0.5 m/s is imperceptible to the driver.
         *
         * TUNE:  0.4–0.6 m/s works for most robots. Wider = smoother but
         *        driver has less control for a longer window.
         */
        public static final double LAUNCH_BLEND_WINDOW_MPS = 0.3;

        /**
         * WHAT:  The joystick magnitude (0–1) that maps to MAX_DRIVE_STATOR_CURRENT.
         *
         * WHY:   We drive the motors in current (torque) rather than speed.
         *        joystickMag × scale = desiredAmps. This keeps traction control
         *        physically meaningful — full stick = full grip-limited force,
         *        not full speed.
         *
         * TUNE:  Equal to MAX_DRIVE_STATOR_CURRENT for a 1:1 mapping.
         *        You can reduce for a softer feel in teleop.
         */
        public static final double JOYSTICK_TO_CURRENT_SCALE = MAX_DRIVE_STATOR_CURRENT;
    }
}
