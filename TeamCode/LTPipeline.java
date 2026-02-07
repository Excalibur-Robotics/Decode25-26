package org.firstinspires.ftc.teamcode.V2;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;L

public class LTPipeline extends OpenCvPipeline{
    Mat hsv = new Mat();
    Mat green_mask = new Mat();
    Mat purple_mask= new Mat();

    double GreenPixels;
    double PurplePixels;

    //Scalars might need tuning
    //Green (RGB range)
    Scalar min_green = new Scalar(35, 80, 50);
    Scalar max_green = new Scalar(85, 255, 255);


    //Purple (RGB range)

    Scalar min_purple = new Scalar(125, 80, 50);
    Scalar max_purple= new Scalar(155, 255, 255);

    public Mat processFrame(Mat input) {
        //Converts RGB to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        //Threshold Green & Purple
        Core.inRange(hsv, min_green, max_green, green_mask);
        Core.inRange(hsv, min_purple, max_purple, purple_mask);

        //Green Pixels
        GreenPixels = Core.countNonZero(green_mask);
        PurplePixels = Core.countNonZero(purple_mask);


        if ( PurplePixels > GreenPixels) {
            //xxx_mask displays xxx pixels as white and every other pixel as black
            return purple_mask;
        } else {
            return green_mask;
        }

    }


}
