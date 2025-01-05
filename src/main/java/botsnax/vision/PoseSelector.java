package botsnax.vision;

import edu.wpi.first.math.Pair;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class PoseSelector implements Supplier<Optional<PoseEstimate>> {
    private final List<Supplier<Optional<PoseEstimate>>> sources;
    private final Function<PoseEstimate, Double> getPoseQuality;

    public PoseSelector(List<Supplier<Optional<PoseEstimate>>> sources, Function<PoseEstimate, Double> getPoseQuality) {
        this.sources = sources;
        this.getPoseQuality = getPoseQuality;
    }

    @Override
    public Optional<PoseEstimate> get() {
        return sources
                .stream()
                .map(Supplier::get)
                .map(estimateIfAny -> estimateIfAny
                        .map(estimate -> new Pair<>(getPoseQuality.apply(estimate), Optional.of(estimate)))
                        .orElse(new Pair<>(-Double.MAX_VALUE, Optional.empty())))
                .min(Comparator.comparing(Pair::getFirst))
                .flatMap(Pair::getSecond);
    }
}
