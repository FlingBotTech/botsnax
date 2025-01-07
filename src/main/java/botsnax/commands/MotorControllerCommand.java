package botsnax.commands;

import botsnax.control.MotorController;
import botsnax.system.motor.MotorState;
import botsnax.system.motor.MotorSystem;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Supplier;

import static edu.wpi.first.hal.HALUtil.getFPGATime;
import static edu.wpi.first.units.Units.Microseconds;

public class MotorControllerCommand extends Command {
    private final MotorSystem motorSystem;
    private final Supplier<MotorController> supplier;
    private Time startTime;

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
        Time time = Microseconds.of(getFPGATime()).minus(startTime);
        MotorState state = motorSystem.getState(time);
        Voltage voltage = supplier.get().calculate(state);

        motorSystem.setVoltage(voltage);
    }
}

