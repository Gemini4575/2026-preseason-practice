package frc.robot.service;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArrayTopic;
import frc.robot.model.MetricName;

public class MetricService {

    private static final int MAX_CACHE_SIZE = 5000;
    private static List<Pair<MetricName, Double>> capturedMetrics = new ArrayList<>();
    private static StringArrayPublisher jsonPublisher;

    public static void init() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("metrics");
        StringArrayTopic jsonTopic = table.getStringArrayTopic("json");
        jsonPublisher = jsonTopic.publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        System.out.println("Topics: " + table.getTopics());
    }

    public static void close() {
        if (jsonPublisher != null) {
            jsonPublisher.close();
        }
    }

    public static void publish(MetricName metricName, double value) {
        if (capturedMetrics.size() < MAX_CACHE_SIZE) {
            capturedMetrics.add(new Pair<>(metricName, value));
        }
    }

    private static String toJson(MetricName metricName, double value) {
        return String.format("{\"metricName\": \"%s\", \"type\": \"double\", \"value\": %f}", metricName, value);
    }

    public static void periodic() {
        if (capturedMetrics.isEmpty()) {
            return;
        }
        var previousList = capturedMetrics;
        capturedMetrics = new ArrayList<>();
        if (jsonPublisher != null) {
            // System.out.println("Publishing metric: " + metricName + " = " + value);
            jsonPublisher
                    .set(previousList.stream().map(v -> toJson(v.getFirst(), v.getSecond())).toArray(String[]::new));
        }
    }

}
