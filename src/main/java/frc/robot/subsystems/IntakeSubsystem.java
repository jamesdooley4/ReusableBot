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
        io.setRollerVoltage(9.0);
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

    // (legacy simple setters removed — use command factories below)

    // Command factories for deploying/stowing using IO metrics (current + velocity)
    public Command deployIntakeCommand() {
        return Commands.sequence(
            // Run deploy voltage until current spike + near-zero velocity or timeout
            run(() -> io.setDeployVoltage(6.0))
                .until(() -> io.getDeployCurrent() > 10.0 && Math.abs(io.getDeployVelocityRadPerSec()) < 0.5)
                .withTimeout(3.0),
            runOnce(() -> io.stopDeployMotor())
        );
    }

    public Command stowIntakeCommand() {
        return Commands.sequence(
            run(() -> io.setDeployVoltage(-7.0))
                .until(() -> io.getDeployCurrent() > 10.0 && Math.abs(io.getDeployVelocityRadPerSec()) < 0.5)
                .withTimeout(3.0),
            runOnce(() -> io.stopDeployMotor())
        );
    }

    public boolean hasPiece() { return io.hasGamePiece(); }
    public State getState() { return state; }

    public Command intake() {
        return Commands.sequence(
            deployIntakeCommand(),
            run(() -> runIntake())
                .until(this::hasPiece),
            Commands.runOnce(this::maintainHold, this)
        );
    }

    public Command outtake(double timeoutSeconds) {
        return run(() -> runOuttake())
            .withTimeout(timeoutSeconds)
            .until(() -> !hasPiece());
    }

    public Command autoIntake() {
        return Commands.sequence(
            deployIntakeCommand(),
            run(() -> runIntake())
                .until(this::hasPiece),
            Commands.runOnce(this::maintainHold, this),
            stowIntakeCommand()
        );
    }

    public Command hold() {
        return run(() -> maintainHold());
    }

    public Command stow() {
        return Commands.sequence(
            Commands.runOnce(this::stopRoller, this),
            stowIntakeCommand()
        );
    }

    public Command clearJam() {
        return Commands.sequence(
            run(() -> runOuttake()).withTimeout(0.5),
            run(() -> runIntake()).withTimeout(0.5)
        );
    }

    public Command stop() {
        return runOnce(() -> stopRoller());
    }
}
