package frc.robot.service;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import frc.robot.model.MetricName;

public class MetricService {

    private static StringPublisher jsonPublisher;

    public static void init() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("metrics");
        StringTopic jsonTopic = table.getStringTopic("json");
        jsonPublisher = jsonTopic.publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
        System.out.println("Topics: " + table.getTopics());
    }

    public static void close() {
        if (jsonPublisher != null) {
            jsonPublisher.close();
        }
    }

    public static void publish(MetricName metricName, double value) {
        if (jsonPublisher != null) {
            // System.out.println("Publishing metric: " + metricName + " = " + value);
            jsonPublisher.set(toJson(metricName, value));
        }
    }

    private static String toJson(MetricName metricName, double value) {
        return String.format("{\"metricName\": \"%s\", \"type\": \"double\", \"value\": %f}", metricName, value);
    }

}
