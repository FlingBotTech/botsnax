package botsnax.system;

public interface VoltageActuator {
    void setVoltage(double voltage);
    double getVoltage();
    void stop();
}
