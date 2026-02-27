package org.firstinspires.ftc.teamcode.V2;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class LTPipeline extends OpenCvPipeline{
    Mat hsv = new Mat();
    Mat green_mask = new Mat();
    Mat purple_mask= new Mat();

    public double GreenPixels;
    public double PurplePixels;
    public static int min_green_R = 0;
    public static int min_green_G = 130;
    public static int min_green_B = 110;
    public static int max_green_R = 120;
    public static int max_green_G = 255;
    public static int max_green_B = 255;

    public static int min_purple_R = 125;
    public static int min_purple_G = 80;
    public static int min_purple_B = 50;
    public static int max_purple_R = 255;
    public static int max_purple_G = 155;
    public static int max_purple_B = 255;

    //Scalars might need tuning
    //Green (RGB range)
    Scalar min_green = new Scalar(min_green_R, min_green_G, min_green_B);
    Scalar max_green = new Scalar(max_green_R, max_green_G, max_green_B);

    //Purple (RGB range)
    Scalar min_purple = new Scalar(min_purple_R, min_purple_G, min_purple_B);
    Scalar max_purple= new Scalar(max_purple_R, max_purple_G, max_purple_B);

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
