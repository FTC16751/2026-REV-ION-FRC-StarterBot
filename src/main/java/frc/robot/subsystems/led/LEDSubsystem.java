package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

/**
 * LEDSubsystem for VOLT (Valkyries Of Leadership and Technology)
 * Controls WS2812B addressable LED strip via roboRIO PWM port.
 *
 * Hardware:
 *   - WS2812B addressable LED strip
 *   - Data line connected to roboRIO PWM port (PWM_PORT)
 *
 * Architecture:
 *   External code calls setState() to set the base game state (SHOOTING, TELEOP, etc.).
 *   periodic() runs a priority cascade of system-level overrides BEFORE the state switch.
 *   Higher-priority conditions early-return so lower conditions and the switch are skipped.
 *
 *   Priority order (highest first):
 *     P1: E-stop          → solid red
 *     P2: FMS data missing → white/red strobe
 *     P3: Shift ending     → yellow/orange urgent wave
 *     P4: Match over       → VOLT lightning bolt
 *     P5: Low battery      → red blink (disabled only)
 *     Normal: state switch
 */
public class LEDSubsystem extends SubsystemBase {

    // ── Hardware Constants ────────────────────────────────────────────────────
    private static final int PWM_PORT   = 0;    // PWM port on roboRIO (change if needed)
    private static final int LED_LENGTH = 77;   // Number of LEDs on your strip

    // ── Pattern Timing ────────────────────────────────────────────────────────
    private static final double BLINK_INTERVAL    = 0.15; // seconds per blink half-cycle
    private static final double FAST_BLINK_INTERVAL = 0.05; // seconds per fast blink half-cycle
    private static final double BREATHE_PERIOD    = 2.0;  // seconds per breathe cycle
    private static final double CHASE_INTERVAL    = 0.05; // seconds per chase step
    private static final double CHASE_AIM_INTERVAL = 0.02; // fast chase for aim-locked

    // ── System Alert Thresholds ───────────────────────────────────────────────

    /** Battery voltage below this → LOW_BATTERY pattern while disabled. */
    private static final double BATTERY_LOW_THRESHOLD = 11.5; // volts

    /**
     * When fewer than this many seconds remain in an ACTIVE shift window,
     * override the normal pattern with an urgent wave to tell the driver: SHOOT NOW.
     */
    private static final double SHIFT_WARN_TIME = 5.0; // seconds

    // ── FRC 2026 Shift Schedule ───────────────────────────────────────────────
    /**
     * Teleop is divided into 6 time windows ("shifts"). Each shift belongs to one
     * alliance's hub (active = you can score; inactive = opponent scores).
     *
     * Shift boundaries (seconds from teleop start):
     *   Shift 0:  0–10s    both alliances active (grace/transition period)
     *   Shift 1: 10–35s    alternating (one alliance active)
     *   Shift 2: 35–60s    alternating
     *   Shift 3: 60–85s    alternating
     *   Shift 4: 85–110s   alternating
     *   Shift 5: 110–140s  both alliances active (endgame)
     *
     * The alliance that LOST auto scoring gets compensation: first active window
     * (shifts 1 and 3). The auto WINNER gets shifts 2 and 4.
     *
     * FMS sends this via getGameSpecificMessage(): 'R'=Red won auto, 'B'=Blue won auto.
     */
    private static final double[] SHIFT_START = {0.0,  10.0, 35.0, 60.0, 85.0,  110.0};
    private static final double[] SHIFT_END   = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

    /** Hub-active schedule for the alliance that gets the FIRST active window (auto loser). */
    private static final boolean[] ACTIVE_FIRST_SCHED   = {true, true,  false, true,  false, true};
    /** Hub-active schedule for the alliance that gets the SECOND active window (auto winner). */
    private static final boolean[] INACTIVE_FIRST_SCHED = {true, false, true,  false, true,  true};

    // ── Hardware Objects ──────────────────────────────────────────────────────
    private final AddressableLED       m_led;
    private final AddressableLEDBuffer m_buffer;

    // ── Named Colors ──────────────────────────────────────────────────────────
    private enum LedColor {
        RED         (255,   0,   0),
        GREEN       (  0, 255,   0),
        WHITE       (255, 255, 255),
        BLUE        (  0, 100, 255),
        PURPLE      ( 75,   0, 130),
        ORANGE      (255, 165,   0),  // target locked / too far warning
        VOLT_ORANGE (255,  80,   0),  // inactive-hub solid / team color
        VOLT_YELLOW (255, 200,   0),  // idle breathe highlight
        DEEP_ORANGE (255,  50,   0),  // low battery blink / alert blink
        DIM_ORANGE  ( 40,  12,   0),  // lightning bolt background
        PINK        (255,  20, 147);  // distinct, unused color for inactive shift

        final int r, g, b;
        LedColor(int r, int g, int b) { this.r = r; this.g = g; this.b = b; }
    }

    // ── AprilTag Overlay State ─────────────────────────────────────────────────
    private boolean m_aprilTagVisible = false;

    // ── Base State (set externally via setState()) ────────────────────────────
    private LEDState m_currentState = LEDState.IDLE;

    /**
     * State-relative timer — reset each time setState() changes the state.
     * Used by blink, breathe, and chase patterns so they start fresh on each state entry.
     */
    private final Timer m_timer = new Timer();
    private int m_chaseIndex = 0;

    // ── Shift / Match Tracking State ──────────────────────────────────────────

    /**
     * Dedicated timer for the FRC 2026 shift schedule.
     * Restarted at teleop enable; never reset mid-match.
     * Its value (in seconds) is used to look up the current shift window.
     */
    private final Timer m_teleopTimer = new Timer();

    private boolean m_wasTeleop    = false; // was teleop active last periodic cycle?
    private boolean m_wasEnabled   = false; // was robot enabled last periodic cycle?

    /**
     * True if OUR alliance gets the first active hub window (i.e., we lost auto scoring).
     * Set once per teleop by parseGameData() when the FMS message arrives.
     */
    private boolean m_firstShiftActive = false;

    /** True once we have successfully parsed the FMS game-specific message this teleop. */
    private boolean m_gameDataParsed   = false;

    /**
     * Latches true when the FMS confirms the match has just ended.
     * Detected by: was in teleop → now disabled → match time ≈ 0.
     * Only fires at real competition, not during practice or simulation.
     */
    private boolean m_matchOver        = false;

    // ─────────────────────────────────────────────────────────────────────────
    // LED States
    // ─────────────────────────────────────────────────────────────────────────
    public enum LEDState {
        /** Robot is idle / disabled */
        IDLE,

        /** Robot is enabled and driving normally */
        TELEOP,

        /** Vision has a target locked */
        TARGET_LOCKED,

        /** Robot is in range AND has target — ready to shoot */
        READY_TO_SHOOT,

        /** Robot is aimed within tolerance of the target */
        AIM_LOCKED,

        /** Robot is actively shooting */
        SHOOTING,

        /** Robot is too far from the hub to score reliably */
        TOO_FAR,

        /** Autonomous mode running */
        AUTO,

        /**
         * Low battery warning.
         * Can be set manually via setState(), or fires automatically as P5 override
         * when disabled and RobotController.getBatteryVoltage() < BATTERY_LOW_THRESHOLD.
         */
        LOW_BATTERY,

        /** Celebratory end-game pattern (use during final 20s countdown, not match-over) */
        END_GAME,

        /** E-stop or fault (P1 override also handles hardware e-stop automatically) */
        FAULT
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────
    public LEDSubsystem() {
        m_led    = new AddressableLED(PWM_PORT);
        m_buffer = new AddressableLEDBuffer(LED_LENGTH);

        m_led.setLength(LED_LENGTH);
        m_led.setData(m_buffer);
        m_led.start();

        m_timer.start(); // state-relative timer always runs; reset on state change
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Periodic — called every robot loop (~20ms)
    //
    // Flow:
    //   1. Update shift/match tracking (timers, edge detection, game data parsing)
    //   2. Priority cascade — high-priority conditions early-return before switch
    //   3. Normal state switch (base game state set by setState())
    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        double periodicStartTime = Timer.getFPGATimestamp();
        try {
            periodicInternal();
        } finally {
            Command currentCommand = this.getCurrentCommand();
            Logger.recordOutput("LEDs/CurrentCommand", currentCommand != null ? currentCommand.getName() : "None");
            Logger.recordOutput("LEDs/PeriodicExecutionTimeMs", (Timer.getFPGATimestamp() - periodicStartTime) * 1000.0);
        }
    }

    private void periodicInternal() {
        boolean isTeleop  = DriverStation.isTeleopEnabled();
        boolean isEnabled = DriverStation.isEnabled();

        // ── 1a. Teleop rising edge: restart shift timer and re-arm game data parsing ──
        // This block fires exactly once per teleop enable.
        if (isTeleop && !m_wasTeleop) {
            m_teleopTimer.restart();  // timer now tracks seconds since teleop start
            m_gameDataParsed = false; // re-parse FMS data for each new teleop period
            m_matchOver      = false; // clear in case of re-enable after a match
        }

        // ── 1b. End-of-match detection: teleop → disabled with FMS time ≈ 0 ───────────
        // We detect the falling edge of "was in teleop, now disabled."
        // getMatchTime() returns remaining seconds when FMS-connected; -1 otherwise.
        // The < 2.0 guard prevents false positives from mid-match disables.
        if (!isEnabled && m_wasEnabled && m_wasTeleop) {
            double matchTime = DriverStation.getMatchTime();
            if (matchTime >= 0.0 && matchTime < 2.0) {
                m_matchOver = true; // real match end confirmed by FMS
            }
        }

        // Store current state for next cycle's edge detection
        m_wasTeleop  = isTeleop;
        m_wasEnabled = isEnabled;

        // ── 1c. Parse FMS game data (once per teleop, when message arrives) ─────────
        // getGameSpecificMessage() may arrive slightly after teleop enable, so we
        // keep retrying until m_gameDataParsed is latched true.
        if (isTeleop && !m_gameDataParsed) parseGameData();

        // -
        // PRIORITY OVERRIDES — highest priority first, each early-returns if triggered
        // -

        // ── P1: E-stop → solid red ────────────────────────────────────────────────
        if (DriverStation.isEStopped()) {
            patternSolid(LedColor.RED);
            applyAndSend();
            return;
        }

        // ── P2: Auto winner not set → white/red fast strobe ──────────────────────
        if (isTeleop && DriverStation.isFMSAttached() && m_teleopTimer.hasElapsed(1.0)
                && DriverStation.getGameSpecificMessage().isEmpty()) {
            patternAlertStrobe();
            applyAndSend();
            return;
        }

        // ── P3: Active shift ending soon → urgent yellow/orange wave ─────────────
        if (isTeleop && DriverStation.isFMSAttached() && isOurShiftActive() && shiftTimeRemaining() < SHIFT_WARN_TIME) {
            patternWave(LedColor.VOLT_YELLOW.r, LedColor.VOLT_YELLOW.g, LedColor.VOLT_YELLOW.b,
                        LedColor.VOLT_ORANGE.r, LedColor.VOLT_ORANGE.g, LedColor.VOLT_ORANGE.b);
            applyAndSend();
            return;
        }

        // ── P4: Match over → VOLT lightning bolt ─────────────────────────────────
        if (m_matchOver) {
            patternLightningBolt();
            applyAndSend();
            return;
        }

        // ── P5: Low battery (disabled only) → red blink ──────────────────────────
        if (DriverStation.isDisabled()
                && RobotController.getBatteryVoltage() < BATTERY_LOW_THRESHOLD) {
            patternBlink(LedColor.RED, FAST_BLINK_INTERVAL);
            applyAndSend();
            return;
        }

        // -----------------------------------------------------------------------
        // NORMAL STATE SWITCH — applies the base state set by setState()
        // All priority overrides above have already early-returned if active.
        // ---------------------------------------------------------------------
        switch (m_currentState) {

            // Disabled idle: pulse between deep orange and yellow (VOLT team colors).
            case IDLE -> patternBreatheColor(
                    LedColor.DEEP_ORANGE.r, LedColor.DEEP_ORANGE.g, LedColor.DEEP_ORANGE.b,
                    LedColor.VOLT_YELLOW.r, LedColor.VOLT_YELLOW.g, LedColor.VOLT_YELLOW.b);

            // Teleop: shift-aware indicator.
            //   Blue/purple wave = our hub is ACTIVE  → score now!
            //   Orange solid     = our hub is INACTIVE → defend, collect, wait
            case TELEOP -> {
                if (isTeleop && isOurShiftActive())
                    patternSolid(LedColor.BLUE);

                   // patternWave(LedColor.BLUE.r,  LedColor.BLUE.g,  LedColor.BLUE.b,
                     //           LedColor.PURPLE.r, LedColor.PURPLE.g, LedColor.PURPLE.b);
                else
                    patternSolid(LedColor.PURPLE);
            }

            // Vision target acquired — blink orange to alert operator
            case TARGET_LOCKED  -> patternBlink(LedColor.ORANGE);

            // In range and aimed — solid green signals "ready to fire"
            case READY_TO_SHOOT -> patternSolid(LedColor.GREEN);

            // Aim PID converged on target — fast green chase shows active tracking
            case AIM_LOCKED     -> patternChaseAim();

            // Currently shooting — green blink for visual confirmation
            case SHOOTING       -> patternBlink(LedColor.GREEN);

            // Too far from hub to score reliably — orange blink warning
            case TOO_FAR        -> patternBlink(LedColor.ORANGE);

            // Autonomous running — smooth blue/purple wave
            case AUTO           -> patternWave(LedColor.BLUE.r,  LedColor.BLUE.g,  LedColor.BLUE.b,
                                               LedColor.PURPLE.r, LedColor.PURPLE.g, LedColor.PURPLE.b);

            // Low battery — manual override (P5 auto-detects while disabled)
            case LOW_BATTERY    -> patternBlink(LedColor.RED, FAST_BLINK_INTERVAL);

            // End-game countdown celebration (during match, last ~20s)
            case END_GAME       -> patternRainbowChase();

            // E-stop state set via setState() — blink red
            case FAULT          -> patternBlink(LedColor.RED);
        }

        applyAndSend();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public State Setter
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Set the LED base state. The priority cascade in periodic() may override this
     * with a higher-priority condition (e-stop, FMS alert, shift warning, etc.).
     *
     * The state is only changed (and timer reset) if it actually differs, to avoid
     * unnecessarily resetting pattern timers.
     *
     * Example: m_leds.setState(LEDSubsystem.LEDState.READY_TO_SHOOT);
     */
    public void setState(LEDState state) {
        if (state != m_currentState) {
            m_currentState = state;
            m_timer.reset(); // restart state-relative timer so patterns begin fresh
            m_chaseIndex   = 0;
        }
    }

    public LEDState getState() {
        return m_currentState;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Shift Tracking Helpers
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Parse the FMS game-specific message to determine our shift schedule.
     * Should be called repeatedly each cycle until m_gameDataParsed is set true.
     *
     * FRC 2026 message format — first character:
     *   'R' = Red alliance won auto scoring → Blue gets first active hub window
     *   'B' = Blue alliance won auto scoring → Red gets first active hub window
     *
     * The auto LOSER receives compensation by getting the first active shifts (1 and 3).
     * Result is stored in m_firstShiftActive which selects the correct schedule array.
     */
    private void parseGameData() {
        String msg = DriverStation.getGameSpecificMessage();
        if (!msg.isEmpty()) {
            Alliance us = DriverStation.getAlliance().orElse(Alliance.Blue);
            char c = msg.charAt(0);
            // 'R' won auto → Blue is the loser → Blue gets first active window
            // 'B' won auto → Red is the loser → Red gets first active window
            m_firstShiftActive = (c == 'R') ? (us == Alliance.Blue) : (us == Alliance.Red);
            m_gameDataParsed   = true;
        }
        // If message is still empty, keep m_gameDataParsed = false and try again next cycle
    }

    /**
     * Returns true if our alliance's hub is currently active based on the FRC 2026
     * shift schedule and the time elapsed since teleop start (m_teleopTimer).
     *
     * Selects ACTIVE_FIRST_SCHED or INACTIVE_FIRST_SCHED based on m_firstShiftActive,
     * then looks up the boolean for the current shift window.
     *
     * Returns false if the timer is past all shift windows (shouldn't happen normally).
     *
     * Callers should guard with "isTeleop &&" to avoid calling this outside teleop.
     */
    private boolean isOurShiftActive() {
        double   t     = m_teleopTimer.get();
        boolean[] sched = m_firstShiftActive ? ACTIVE_FIRST_SCHED : INACTIVE_FIRST_SCHED;
        for (int i = 0; i < SHIFT_START.length; i++) {
            if (t >= SHIFT_START[i] && t < SHIFT_END[i]) return sched[i];
        }
        return false;
    }

    /**
     * Returns the number of seconds remaining in the current shift window.
     * Used by P3 to decide when to show the end-of-shift warning pattern.
     * Returns 0.0 if the timer is past all shift windows.
     */
    private double shiftTimeRemaining() {
        double t = m_teleopTimer.get();
        for (int i = 0; i < SHIFT_START.length; i++) {
            if (t >= SHIFT_START[i] && t < SHIFT_END[i]) return SHIFT_END[i] - t;
        }
        return 0.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Patterns
    // ─────────────────────────────────────────────────────────────────────────

    // ─────────────────────────────────────────────────────────────────────────
    // Public AprilTag Overlay Setter
    // ─────────────────────────────────────────────────────────────────────────

    /** Call with true when Limelight sees an AprilTag; false when it loses sight. */
    public void setAprilTagVisible(boolean visible) {
        m_aprilTagVisible = visible;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Send Helper — applies AprilTag overlay then pushes buffer to strip
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Applies the AprilTag indicator overlay (pixels 0–2 and 74–76 → green)
     * on top of whatever pattern is already in the buffer, then sends to strip.
     * Call instead of m_led.setData(m_buffer) everywhere in periodic().
     */
    private void applyAndSend() {
        if (m_aprilTagVisible) {
            m_buffer.setRGB(0, 0, 255, 0);
            m_buffer.setRGB(1, 0, 255, 0);
            m_buffer.setRGB(2, 0, 255, 0);
            m_buffer.setRGB(74, 0, 255, 0);
            m_buffer.setRGB(75, 0, 255, 0);
            m_buffer.setRGB(76, 0, 255, 0);
        }
        m_led.setData(m_buffer);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // LedColor overloads — convenience wrappers for single-color patterns
    // ─────────────────────────────────────────────────────────────────────────

    private void patternSolid(LedColor c)   { patternSolid(c.r, c.g, c.b); }
    private void patternBlink(LedColor c)   { patternBlink(c.r, c.g, c.b); }
    private void patternBlink(LedColor c, double interval) { patternBlink(c.r, c.g, c.b, interval); }
    private void patternBreathe(LedColor c) { patternBreathe(c.r, c.g, c.b); }

    // ─────────────────────────────────────────────────────────────────────────
    // Patterns
    // ─────────────────────────────────────────────────────────────────────────

    /** Fill entire strip with one solid color. */
    private void patternSolid(int r, int g, int b) {
        for (int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setRGB(i, r, g, b);
        }
    }

    /** Blink the entire strip on/off at the specified interval. */
    private void patternBlink(int r, int g, int b, double interval) {
        boolean on = (int)(m_timer.get() / interval) % 2 == 0;
        patternSolid(on ? r : 0, on ? g : 0, on ? b : 0);
    }

    /**
     * Blink the entire strip on/off at BLINK_INTERVAL.
     * Uses the state-relative m_timer (resets when setState() is called).
     */
    private void patternBlink(int r, int g, int b) {
        patternBlink(r, g, b, BLINK_INTERVAL);
    }

    /**
     * Smooth sine-wave breathing — fades between the given color and black.
     * Kept for compatibility; see patternBreatheColor() for two-color version.
     */
    private void patternBreathe(int r, int g, int b) {
        double phase      = (m_timer.get() % BREATHE_PERIOD) / BREATHE_PERIOD;
        double brightness = (Math.sin(phase * 2 * Math.PI - Math.PI / 2) + 1) / 2;
        patternSolid(
            (int)(r * brightness),
            (int)(g * brightness),
            (int)(b * brightness));
    }

    /**
     * Two-color breathing — smoothly interpolates between c1 and c2 (no fade to black).
     * Used for IDLE to keep the VOLT orange/yellow theme visible at all brightness levels.
     *
     * Example: patternBreatheColor(255, 50, 0, 255, 200, 0)
     *   → pulses between deep orange and warm yellow
     */
    private void patternBreatheColor(int r1, int g1, int b1, int r2, int g2, int b2) {
        double phase = (m_timer.get() % BREATHE_PERIOD) / BREATHE_PERIOD;
        double ratio  = (Math.sin(phase * 2 * Math.PI - Math.PI / 2) + 1) / 2; // 0.0 → 1.0
        patternSolid(
            (int)(r1 * (1 - ratio) + r2 * ratio),
            (int)(g1 * (1 - ratio) + g2 * ratio),
            (int)(b1 * (1 - ratio) + b2 * ratio));
    }

    /**
     * Sinusoidal wave that scrolls across the strip, smoothly blending c1 and c2.
     *
     * Uses Timer.getTimestamp() (wall clock, never reset) so the wave is always
     * continuous — no jump when entering this pattern or switching states.
     *
     * The wave completes about one full cycle across the strip at approximately
     * 3/(2π) Hz (~0.48 cycles/second). Increase the 3.0 multiplier for a faster wave.
     *
     * Adapted from Mechanical Advantage 2026 Leds.java.
     */
    private void patternWave(int r1, int g1, int b1, int r2, int g2, int b2) {
        double t = Timer.getTimestamp(); // wall clock — immune to state resets
        for (int i = 0; i < m_buffer.getLength(); i++) {
            // Phase advances with pixel index (spatial) and decreases with time (motion)
            double phase = (2.0 * Math.PI * i / m_buffer.getLength()) - (t * 3.0);
            double ratio  = (Math.sin(phase) + 1.0) / 2.0; // 0.0 → 1.0
            m_buffer.setRGB(i,
                (int)(r1 * ratio + r2 * (1 - ratio)),
                (int)(g1 * ratio + g2 * (1 - ratio)),
                (int)(b1 * ratio + b2 * (1 - ratio)));
        }
    }

    /**
     * Fast white/red alternating strobe — highest-urgency alert pattern.
     * Used when the FMS game-specific message is missing after 1s of teleop.
     * Drive team must manually confirm the shift schedule before shooting.
     *
     * Uses Timer.getTimestamp() so strobe is independent of all state timers.
     * Alternates every 0.1s (5 Hz).
     */
    private void patternAlertStrobe() {
        boolean showWhite = ((int)(Timer.getTimestamp() / 0.1)) % 2 == 0;
        // White = (255,255,255), Red = (255,0,0) — only green/blue channels change
        patternSolid(255, showWhite ? 255 : 0, showWhite ? 255 : 0);
    }

    /**
     * Two-color moving chase — one "head" pixel moves through the strip.
     * Available for future use; currently unused (AUTO uses wave instead).
     */
    private void patternChase(int r1, int g1, int b1, int r2, int g2, int b2) {
        if (m_timer.get() > CHASE_INTERVAL) {
            m_chaseIndex = (m_chaseIndex + 1) % m_buffer.getLength();
            m_timer.reset();
        }
        for (int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setRGB(i, i == m_chaseIndex ? r1 : r2,
                               i == m_chaseIndex ? g1 : g2,
                               i == m_chaseIndex ? b1 : b2);
        }
    }

    /** Fast single-pixel green chase — indicates aim is locked on target. */
    private void patternChaseAim() {
        if (m_timer.get() > CHASE_AIM_INTERVAL) {
            m_chaseIndex = (m_chaseIndex + 1) % m_buffer.getLength();
            m_timer.reset();
        }
        for (int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setRGB(i, 0, i == m_chaseIndex ? 255 : 30, 0);
        }
    }

    /** Full rainbow cycle scrolling across the strip — end-game celebration. */
    private void patternRainbowChase() {
        int offset = (int)(m_timer.get() * 50) % 180;
        for (int i = 0; i < m_buffer.getLength(); i++) {
            int hue = (offset + (i * 180 / m_buffer.getLength())) % 180;
            m_buffer.setHSV(i, hue, 255, 128);
        }
    }

    /**
     * VOLT post-match lightning bolt celebration.
     *
     * A bright orange→yellow bolt sweeps the strip from end to end repeatedly,
     * referencing the team name (VOLT) and team colors (orange + yellow).
     *
     * Cycle (1.0s total):
     *   0.00s → 0.35s  bolt sweeps from pixel 0 to pixel N
     *   0.35s → 1.00s  dim orange background (pause between bolts)
     *
     * The bolt gradient:
     *   Tip (front): nearly white-yellow — bright, high energy
     *   Body:        warm yellow fading to orange
     *   Tail:        dims to nothing behind the bolt
     *
     * Uses Timer.getTimestamp() (wall clock) so the animation runs continuously
     * and is independent of state timers.
     *
     * To tune the visual: adjust BOLT_DURATION (speed), CYCLE_TIME (gap between bolts),
     * and BOLT_LENGTH (how long the gradient trail is).
     */
    private void patternLightningBolt() {
        final double BOLT_DURATION = 0.35; // seconds for one end-to-end sweep
        final double CYCLE_TIME    = 1.0;  // seconds per full bolt cycle (sweep + pause)
        final int    BOLT_LENGTH   = 18;   // pixels — length of the fading gradient trail

        double t        = Timer.getTimestamp();
        double progress = (t % CYCLE_TIME) / BOLT_DURATION; // 0→1 during sweep, >1 during pause

        // Always paint a dim orange base first — visible during the pause between bolts
        patternSolid(40, 12, 0); // dim deep orange (team color background)

        if (progress <= 1.0) {
            // Bolt head position: advances linearly from pixel 0 to pixel N during the sweep
            int headPos = (int)(progress * m_buffer.getLength());

            // Draw the gradient bolt from head backward (tail trails behind the sweep)
            for (int i = 0; i < BOLT_LENGTH; i++) {
                int pos = headPos - i; // pixels behind the head
                if (pos < 0 || pos >= m_buffer.getLength()) continue;

                // fadeRatio: 1.0 at the tip (i=0), approaches 0 at the tail (i=BOLT_LENGTH-1)
                double fadeRatio = 1.0 - (double) i / BOLT_LENGTH;

                // Color gradient:
                //   Tip   → nearly white-yellow (high r, high g, small b for extra brightness)
                //   Body  → warm yellow-orange (r stays high, g drops, b disappears)
                //   Tail  → fades to dim orange then nothing
                int r = (int)(255 * fadeRatio);             // full red at tip, fades out
                int g = (int)(200 * fadeRatio * fadeRatio); // green fades faster → more orange in tail
                int b = (int)(60  * fadeRatio * fadeRatio); // tiny blue at very tip adds white "flash"
                m_buffer.setRGB(pos, r, g, b);
            }
        }
    }
}
