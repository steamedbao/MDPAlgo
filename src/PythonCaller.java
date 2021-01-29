import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

public class PythonCaller {

	public static void main(String[] args) throws IOException 
	{
		String pythonScriptPath = "C:/tensorflow1/models/research/object_detection/Object_detection_imageTest.py";
		String[] cmd = new String[2];
		cmd[0] = "python "; 
		cmd[1] = pythonScriptPath;
		Runtime rt = Runtime.getRuntime();
		Process pr = rt.exec(cmd);
		BufferedReader bfr = new BufferedReader(new InputStreamReader(pr.getInputStream()));
		String line = "";
		while((line = bfr.readLine()) != null) 
		{
			System.out.println(line);
		}
	}

}
