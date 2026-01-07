package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeSubsystem extends SubsystemBase {
    public enum State {
        IDLE,
        INTAKING,
        OUTTAKING,
        HOLDING,
        JAM_CLEARING
    }

    private final IntakeIO io;
    private State state = State.IDLE;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    private void runIntake() {
        state = State.INTAKING;
        io.setRollerVoltage(6.0);
    }

    private void runOuttake() {
        state = State.OUTTAKING;
        io.setRollerVoltage(-6.0);
    }

    private void maintainHold() {
        state = State.HOLDING;
        io.setRollerVoltage(2.0);
    }

    private void stopRoller() {
        state = State.IDLE;
        io.stopRoller();
    }

    private void deployIntake() { io.setDeployed(true); }
    private void stowIntake() { io.setDeployed(false); }

    public boolean hasPiece() { return io.hasGamePiece(); }
    public State getState() { return state; }

    public Command intake() {
        return Commands.sequence(
            Commands.runOnce(this::deployIntake, this),
            Commands.run(() -> runIntake(), this)
                .until(this::hasPiece),
            Commands.runOnce(this::maintainHold, this)
        );
    }

    public Command outtake(double timeoutSeconds) {
        return Commands.run(() -> runOuttake(), this)
            .withTimeout(timeoutSeconds)
            .until(() -> !hasPiece());
    }

    public Command autoIntake() {
        return Commands.sequence(
            Commands.runOnce(this::deployIntake, this),
            Commands.run(() -> runIntake(), this)
                .until(this::hasPiece),
            Commands.runOnce(this::maintainHold, this),
            Commands.runOnce(this::stowIntake, this)
        );
    }

    public Command hold() {
        return Commands.run(() -> maintainHold(), this);
    }

    public Command stow() {
        return Commands.sequence(
            Commands.runOnce(this::stopRoller, this),
            Commands.runOnce(this::stowIntake, this)
        );
    }

    public Command clearJam() {
        return Commands.sequence(
            Commands.run(() -> runOuttake(), this).withTimeout(0.5),
            Commands.run(() -> runIntake(), this).withTimeout(0.5)
        );
    }
}
