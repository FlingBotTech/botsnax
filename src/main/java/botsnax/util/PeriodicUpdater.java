package botsnax.util;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;
import java.util.function.Function;

public abstract class PeriodicUpdater<T extends Updatable> {
    private final double updateFrequency;
    private final Thread thread;
    private final ReadWriteLock lock = new ReentrantReadWriteLock();
    private final AtomicBoolean running;
    private final T updatable;

    public PeriodicUpdater(double updateFrequency, T updatable) {
        this.updateFrequency = updateFrequency;
        this.running = new AtomicBoolean(false);
        this.thread = new Thread(this :: run);
        this.updatable = updatable;

        thread.setDaemon(true);
    }

    public double getUpdateFrequency() {
        return updateFrequency;
    }

    private void run() {
        init();

        while (running.get()) {
            waitOneCycle();
            apply(Updatable :: update);
        }
    }

    public void apply(Consumer<T> consumer) {
        try {
            lock.writeLock().lock();
            consumer.accept(updatable);
        } finally {
            lock.writeLock().unlock();
        }
    }

    public <R> R query(Function<T, R> f) {
        try {
            lock.readLock().lock();
            return f.apply(updatable);
        } finally {
            lock.readLock().unlock();
        }
    }

    protected abstract void init();
    protected abstract void waitOneCycle();

    public void start() {
        running.set(true);
        thread.start();
    }

    public void stop(final long millis) throws InterruptedException {
        running.set(false);
        thread.join(millis);
    }
}
