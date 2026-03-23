package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;

/**
 * SetLEDStateCommand
 *
 * A simple instant command that sets the LED subsystem to a given state.
 * Use this in button bindings, auto sequences, or alongside other commands.
 *
 * Example usage in RobotContainer:
 *   new JoystickButton(m_driverController, XboxController.Button.kA.value)
 *       .whileTrue(new SetLEDStateCommand(m_leds, LEDState.READY_TO_SHOOT));
 */
public class SetLEDStateCommand extends Command {

    private final LEDSubsystem m_leds;
    private final LEDState     m_state;

    public SetLEDStateCommand(LEDSubsystem leds, LEDState state) {
        m_leds  = leds;
        m_state = state;
        addRequirements(leds);
    }

    @Override
    public void initialize() {
        m_leds.setState(m_state);
    }

    @Override
    public boolean isFinished() {
        return true; // Instant command — sets state and exits immediately
    }
}
