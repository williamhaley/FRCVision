import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.VideoSource;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;

public class VisionApplication {
  private final VideoSource inputCamera;
  private final CvSource outputStream;
  private final String networkTablesKey;
  private final GripPipeline gripPipeline;
  private CvSink inputSink;

  private final Mat mat = new Mat();
  private final Scalar rectangleColor = new Scalar(255, 0, 255);
  private final int rectangleBorderThickness = 4;

  /**
   * Create a new VisionApplicaton using the provided input, output, and pipeline.
   * @param inputCamera
   * @param outputStream
   */
  public VisionApplication(VideoSource inputCamera, CvSource outputStream, String networkTablesKey, GripPipeline gripPipeline) {
    this.inputCamera = inputCamera;
    this.outputStream = outputStream;
    this.networkTablesKey = networkTablesKey;
    this.gripPipeline = gripPipeline;

    this.inputSink = CameraServer.getInstance().getVideo(inputCamera);
  }

  /**
   * Start a thread where we will listen for updates the vision pipeline.
   * 
   * https://docs.wpilib.org/en/stable/docs/software/vision-processing/grip/using-generated-code-in-a-robot-program.html
   */
  public void start() {
    VisionThread visionThread = new VisionThread(this.inputCamera, this.gripPipeline, pipelineOutput -> {
        if (pipelineOutput.filterContoursOutput().isEmpty()) {
            return;
        }
    
        // Based on the output of the pipeline, create a rect around our object.
        Rect newRect = Imgproc.boundingRect(pipelineOutput.filterContoursOutput().get(0));
        
        this.drawRectOnObject(newRect);
    });

    // If it's a daemon, it won't prevent the JVM from exiting. Good for throwaway work.
    // If the thread was writing to disk, this would be bad.
    visionThread.setDaemon(true);
    visionThread.start();
  }

  /**
   * Draw a rectangle based on the inputSink and the bounding rect we calculated
   * from the GRIP pipeline.
   * 
   * https://github.com/wpilibsuite/allwpilib/blob/master/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/intermediatevision/Robot.java
   */
  private void drawRectOnObject(Rect rect) {
    // There's no frame available from the input.
    if (this.inputSink.grabFrame(mat) == 0) {
      System.out.printf("error grabbing frame '%s'\n", inputSink.getError());
      return;
    }

    Point topLeft = rect.tl();
    Point bottomRight = rect.br();
    double x = topLeft.x;
    double y = topLeft.y;
    double width = bottomRight.x - x;
    double height = bottomRight.y - y;

    // This will appear in the Pi's logs, but may be rather noisy.
    System.out.printf("draw rect at (%s, %s) with dimensions (%f, %f)\n", x, y, width, height);

    // Tell network tables about the newest data.
    NetworkTableInstance
        .getDefault()
        .getEntry("BallPosition")
        .setString(rect.toString());

    Imgproc.rectangle(mat, topLeft, bottomRight, rectangleColor, rectangleBorderThickness);

    this.outputStream.putFrame(mat);
  }
}
