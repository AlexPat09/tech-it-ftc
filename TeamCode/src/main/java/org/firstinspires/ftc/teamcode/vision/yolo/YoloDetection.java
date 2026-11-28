package org.firstinspires.ftc.teamcode.vision.yolo;

import java.util.Locale;

/**
 * One object found by YOLO26 in a camera frame.
 *
 * <p>Coordinates are camera-frame pixels: (0, 0) is the top-left corner of the frame, x grows to the
 * right and y grows downward, exactly like the VisionPortal camera stream.
 */
public final class YoloDetection {

    /** Index of the class in the model's label list. */
    public final int classId;
    /** Class name, e.g. "balls". */
    public final String label;
    /** Model confidence, 0..1. */
    public final float confidence;

    /** Bounding box edges, in camera-frame pixels. */
    public final float left, top, right, bottom;

    /** Angle from the camera's optical axis to the box center, in degrees. Positive = right of center. */
    public final double horizontalAngleDeg;
    /** Angle from the camera's optical axis to the box center, in degrees. Positive = below center. */
    public final double verticalAngleDeg;

    public YoloDetection(int classId, String label, float confidence,
                         float left, float top, float right, float bottom,
                         double horizontalAngleDeg, double verticalAngleDeg) {
        this.classId = classId;
        this.label = label;
        this.confidence = confidence;
        this.left = left;
        this.top = top;
        this.right = right;
        this.bottom = bottom;
        this.horizontalAngleDeg = horizontalAngleDeg;
        this.verticalAngleDeg = verticalAngleDeg;
    }

    public float centerX() { return (left + right) * 0.5f; }
    public float centerY() { return (top + bottom) * 0.5f; }
    public float width()   { return right - left; }
    public float height()  { return bottom - top; }

    @Override
    public String toString() {
        return String.format(Locale.US, "%s %.0f%% center=(%.0f,%.0f) size=%.0fx%.0f angle=%+.1f deg",
                label, confidence * 100, centerX(), centerY(), width(), height(), horizontalAngleDeg);
    }
}
