package frc.robot.utilities.commands;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ParallelRunAllDoneCommandGroup extends Command {
    // maps commands in this composition to whether they are still running
    // LinkedHashMap guarantees we iterate over commands in the order they were added (Note that
    // changing the value associated with a command does NOT change the order)
    private final Map<Command, Boolean> m_commands = new LinkedHashMap<>();
    private boolean m_runWhenDisabled = true;
    private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

    /**
     * Creates a new ParallelRunAllDoneCommandGroup. The given commands will be executed simultaneously. The
     * command composition will finish when all commands are finished. If the composition is
     * interrupted, all commands will be interrupted.
     *
     * @param commands the commands to include in this composition.
     */
    @SuppressWarnings("this-escape")
    public ParallelRunAllDoneCommandGroup(Command... commands) {
        addCommands(commands);
    }

    /**
     * Adds the given commands to the group.
     *
     * @param commands Commands to add to the group.
     */
    public final void addCommands(Command... commands) {
        if (m_commands.containsValue(true)) {
            throw new IllegalStateException(
                "Commands cannot be added to a composition while it's running");
        }

        CommandScheduler.getInstance().registerComposedCommands(commands);

        for (Command command : commands) {
            if (!Collections.disjoint(command.getRequirements(), getRequirements())) {
                throw new IllegalArgumentException(
                    "Multiple commands in a parallel composition cannot require the same subsystems");
            }
            m_commands.put(command, false);
            addRequirements(command.getRequirements());
            m_runWhenDisabled &= command.runsWhenDisabled();
            if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                m_interruptBehavior = InterruptionBehavior.kCancelSelf;
            }
        }
    }

    @Override
    public final void initialize() {
        for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
            commandRunning.getKey().initialize();
            commandRunning.setValue(true);
        }
    }

    @Override
    public final void execute() {
        for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
            if (commandRunning.getValue()) {
                if (commandRunning.getKey().isFinished()) {
                    commandRunning.setValue(false);
                }
            }
            commandRunning.getKey().execute();
        }
    }

    @Override
    public final void end(boolean interrupted) {
        if (interrupted) {
            for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
                commandRunning.getKey().end(commandRunning.getValue());
            }
        }
    }

    @Override
    public final boolean isFinished() {
        return !m_commands.containsValue(true);
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_runWhenDisabled;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return m_interruptBehavior;
    }
}
