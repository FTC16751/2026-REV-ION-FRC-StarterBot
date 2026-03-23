package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LEDSubsystem for VOLT (Valkyries Of Leadership and Technology)
 * Controls WS2812B addressable LED strip via roboRIO PWM port.
 *
 * Hardware:
 *   - BTF-Lighting WS2812B strip (3.28ft / ~60 LEDs)
 *   - Powered by CTRE VRM 5V/500mA output
 *   - Data line connected to roboRIO PWM port
 */
public class LEDSubsystem extends SubsystemBase {

    // ── Hardware Constants ────────────────────────────────────────────────────
    private static final int PWM_PORT   = 0;   // PWM port on roboRIO (change if needed)
    private static final int LED_LENGTH = 144;  // Number of LEDs on your strip

    // ── Timing ────────────────────────────────────────────────────────────────
    private static final double BLINK_INTERVAL   = 0.15; // seconds per blink half-cycle
    private static final double BREATHE_PERIOD   = 2.0;  // seconds for one breathe cycle
    private static final double CHASE_INTERVAL     = 0.05; // seconds per chase step
    private static final double CHASE_AIM_INTERVAL = 0.02; // fast chase for aim-locked

    // ── Hardware Objects ──────────────────────────────────────────────────────
    private final AddressableLED       m_led;
    private final AddressableLEDBuffer m_buffer;

    // ── State ─────────────────────────────────────────────────────────────────
    private LEDState m_currentState = LEDState.IDLE;
    private final Timer m_timer = new Timer();
    private int m_chaseIndex = 0;

    // ─────────────────────────────────────────────────────────────────────────
    // LED States — add or rename these to match your robot's game conditions
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

        /** Robot is too close to the goal */
        TOO_CLOSE,

        /** Autonomous mode running */
        AUTO,

        /** Low battery warning */
        LOW_BATTERY,

        /** Celebratory end-game pattern */
        END_GAME,

        /** E-stop or fault */
        FAULT
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────
    public LEDSubsystem() {
        m_led = new AddressableLED(PWM_PORT);
        m_buffer = new AddressableLEDBuffer(LED_LENGTH);

        m_led.setLength(LED_LENGTH);
        m_led.setData(m_buffer);
        m_led.start();

        m_timer.start();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Periodic — called every robot loop (~20ms)
    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        switch (m_currentState) {
            case IDLE          -> patternBreathe(255, 80, 0);          // Orange breathe
            case TELEOP        -> patternSolid(255, 80, 0);         // Orange solid
            case TARGET_LOCKED -> patternBlink(255, 165, 0);         // Orange blink
            case READY_TO_SHOOT-> patternSolid(0, 255, 0);           // Green solid
            case AIM_LOCKED    -> patternChaseAim();                  // Fast green chase
            case SHOOTING      -> patternBlink(0, 255, 0);           // Green blink
            case TOO_CLOSE     -> patternSolid(255, 0, 0);           // Red solid
            case AUTO          -> patternChase(0, 100, 255, 75, 0, 130); // Blue/purple chase
            case LOW_BATTERY   -> patternBlink(255, 50, 0);          // Orange-red blink
            case END_GAME      -> patternRainbowChase();              // Rainbow party
            case FAULT         -> patternBlink(255, 0, 0);           // Red blink
        }
        m_led.setData(m_buffer);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public State Setter
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Set the LED state. Call this from your commands or RobotContainer.
     * Example: m_leds.setState(LEDSubsystem.LEDState.READY_TO_SHOOT);
     */
    public void setState(LEDState state) {
        if (state != m_currentState) {
            m_currentState = state;
            m_timer.reset();
            m_chaseIndex = 0;
        }
    }

    public LEDState getState() {
        return m_currentState;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Patterns
    // ─────────────────────────────────────────────────────────────────────────

    /** Fill entire strip with one solid color. */
    private void patternSolid(int r, int g, int b) {
        for (int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setRGB(i, r, g, b);
        }
    }

    /**
     * Blink the entire strip on/off at BLINK_INTERVAL.
     * @param r red   (0–255)
     * @param g green (0–255)
     * @param b blue  (0–255)
     */
    private void patternBlink(int r, int g, int b) {
        boolean on = (int)(m_timer.get() / BLINK_INTERVAL) % 2 == 0;
        patternSolid(on ? r : 0, on ? g : 0, on ? b : 0);
    }

    /**
     * Smooth sine-wave brightness breathing effect.
     * @param r red   (0–255) at peak brightness
     * @param g green (0–255) at peak brightness
     * @param b blue  (0–255) at peak brightness
     */
    private void patternBreathe(int r, int g, int b) {
        double phase = (m_timer.get() % BREATHE_PERIOD) / BREATHE_PERIOD; // 0.0 – 1.0
        double brightness = (Math.sin(phase * 2 * Math.PI - Math.PI / 2) + 1) / 2; // 0.0 – 1.0
        patternSolid(
            (int)(r * brightness),
            (int)(g * brightness),
            (int)(b * brightness)
        );
    }

    /**
     * Two-color moving chase pattern — one "head" pixel moves through the strip.
     * Great for autonomous to show the robot is actively running a path.
     */
    private void patternChase(int r1, int g1, int b1, int r2, int g2, int b2) {
        if (m_timer.get() > CHASE_INTERVAL) {
            m_chaseIndex = (m_chaseIndex + 1) % m_buffer.getLength();
            m_timer.reset();
        }
        for (int i = 0; i < m_buffer.getLength(); i++) {
            if (i == m_chaseIndex) {
                m_buffer.setRGB(i, r1, g1, b1); // head pixel
            } else {
                m_buffer.setRGB(i, r2, g2, b2); // trail color
            }
        }
    }

    /** Fast green chase — indicates robot is aimed at target within tolerance. */
    private void patternChaseAim() {
        if (m_timer.get() > CHASE_AIM_INTERVAL) {
            m_chaseIndex = (m_chaseIndex + 1) % m_buffer.getLength();
            m_timer.reset();
        }
        for (int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setRGB(i, 0, i == m_chaseIndex ? 255 : 30, 0);
        }
    }

    /**
     * Full rainbow cycle that shifts over time — end-game celebration.
     */
    private void patternRainbowChase() {
        int offset = (int)(m_timer.get() * 50) % 180;
        for (int i = 0; i < m_buffer.getLength(); i++) {
            int hue = (offset + (i * 180 / m_buffer.getLength())) % 180;
            m_buffer.setHSV(i, hue, 255, 128);
        }
    }
}
