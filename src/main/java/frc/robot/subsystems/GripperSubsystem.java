package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class GripperSubsystem extends SubsystemBase {
    public enum State {
        IDLE,
        INTAKING,
        HOLDING,
        RELEASING
    }

    private final GripperIO io;
    private State state = State.IDLE;

    public GripperSubsystem(GripperIO io) {
        this.io = io;
    }

    public void intake() {
        state = State.INTAKING;
        io.setRollerVoltage(6.0);
        io.setOpen(true);
    }

    public void hold() {
        state = State.HOLDING;
        io.setRollerVoltage(2.0);
        io.setOpen(false);
    }

    public void release() {
        state = State.RELEASING;
        io.setRollerVoltage(-6.0);
        io.setOpen(true);
    }

    public void stop() {
        state = State.IDLE;
        io.stopRoller();
    }

    public boolean hasPiece() {
        return io.hasGamePiece();
    }

    public State getState() {
        return state;
    }

    // --- Command factories ---

    /**
     * Run the gripper intake until a piece is detected, then hold.
     */
    public Command intakeWithGripperCommand() {
        return Commands.sequence(
            // run intake until hasPiece
            run(() -> intake()).until(this::hasPiece),
            // then maintain hold
            runOnce(() -> hold())
        );
    }

    /**
     * Manual control command for the gripper roller. Positive volts intakes,
     * negative volts outtakes. Sets state appropriately while running.
     */
    public Command manualControlCommand(DoubleSupplier voltsSupplier) {
        return run(() -> {
            double v = voltsSupplier.getAsDouble();
            if (v > 0.0) {
                state = State.INTAKING;
                io.setRollerVoltage(v);
                io.setOpen(true);
            } else if (v < 0.0) {
                state = State.RELEASING;
                io.setRollerVoltage(v);
                io.setOpen(true);
            } else {
                state = State.IDLE;
                io.stopRoller();
            }
        });
    }
}
