package botsnax.commands;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import botsnax.control.MotorController;
import botsnax.system.motor.MotorSystem;
import botsnax.system.motor.MotorState;

import java.util.function.Supplier;

import static edu.wpi.first.hal.HALUtil.getFPGATime;
import static edu.wpi.first.units.Units.Microseconds;

public class MotorControllerCommand extends Command {
    private final MotorSystem motorSystem;
    private final Supplier<MotorController> supplier;
    private Measure<Time> startTime;

    public MotorControllerCommand(MotorSystem motorSystem, Supplier<MotorController> supplier, Subsystem ... requirements) {
        this.motorSystem = motorSystem;
        this.supplier = supplier;

        addRequirements(requirements);
    }

    public MotorControllerCommand(MotorSystem motorSystem, MotorController controller, Subsystem ... requirements) {
        this(motorSystem, () -> controller, requirements);
    }

    @Override
    public void initialize() {
        startTime = Microseconds.of(getFPGATime());
    }

    @Override
    public void execute() {
        Measure<Time> time = Microseconds.of(getFPGATime()).minus(startTime);
        MotorState state = motorSystem.getState(time);
        double voltage = supplier.get().calculate(state);

        motorSystem.setVoltage(voltage);
    }
}

