package botsnax.system.encoder;

public interface AbsoluteEncoder extends Encoder {
    void applySimZeroingValue(double value);
    double getZeroingValue();
}
