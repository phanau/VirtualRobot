package virtual_robot.util.color;

//import com.sun.javafx.util.Utils;
//import com.sun.javafx.Utils;
//import java.awt.Color;



public class Color {
    public static void RGBtoHSV(int red, int green, int blue, float[] hsv) {
        RGBtoHSB(red, green, blue, hsv);
    }


    /**
     * 741:    * Converts from the RGB (red, green, blue) color model to the HSB (hue,
     * 742:    * saturation, brightness) color model. If the array is null, a new one
     * 743:    * is created, otherwise it is recycled. The results will be in the range
     * 744:    * 0.0-1.0 if the inputs are in the range 0-255.
     * 745:    *
     * 746:    * @param red the red part of the RGB value
     * 747:    * @param green the green part of the RGB value
     * 748:    * @param blue the blue part of the RGB value
     * 749:    * @param array an array for the result (at least 3 elements), or null
     * 750:    * @return the array containing HSB value
     * 751:    * @throws ArrayIndexOutOfBoundsException of array is too small
     * 752:    * @see #getRGB()
     * 753:    * @see #Color(int)
     * 754:    * @see ColorModel#getRGBdefault()
     * 755:
     */
    public static float[] RGBtoHSB(int red, int green, int blue, float array[]) {
        if (array == null)
            array = new float[3];
     // Calculate brightness.
        int min;
        int max;
        if (red < green) {
            min = red;
            max = green;
        } else {
            min = green;
            max = red;
        }
        if (blue > max)
            max = blue;
        else if (blue < min)
            min = blue;
        array[2] = max / 255f;
        // Calculate saturation.
        if (max == 0)
            array[1] = 0;
        else
            array[1] = ((float) (max - min)) / ((float) max);
        // Calculate hue.
        if (array[1] == 0)
            array[0] = 0;
        else {
            float delta = (max - min) * 6;
            if (red == max)
                array[0] = (green - blue) / delta;
            else if (green == max)
                array[0] = 1f / 3 + (blue - red) / delta;
            else
                array[0] = 2f / 3 + (red - green) / delta;
            if (array[0] < 0)
                array[0]++;
        }
        return array;
    }

}