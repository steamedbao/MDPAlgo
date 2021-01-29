package object_detection;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

public class ImageTest {
    public static void main(String[] args) throws IOException {
        Process p = Runtime.getRuntime().exec("python3 /Users/hannancao/Desktop/CZ3004/untitled/src/object_detection/Object_detection_imageTest.py");
        BufferedReader stdInput = new BufferedReader(new
                InputStreamReader(p.getInputStream()));

        BufferedReader stdError = new BufferedReader(new
                InputStreamReader(p.getErrorStream()));

        // read the output from the command
        System.out.println("Here is the standard output of the command:\n");
        String s = null;
        while ((s = stdInput.readLine()) != null) {
            System.out.println(s);
        }

        // read any errors from the attempted command
        System.out.println("Here is the standard error of the command (if any):\n");
        while ((s = stdError.readLine()) != null) {
            System.out.println(s);
        }
    }
}

