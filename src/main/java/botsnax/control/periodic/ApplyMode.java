package botsnax.control.periodic;

public interface ApplyMode<ValueT, SystemT> {
    void apply(ValueT value, SystemT system);
}
