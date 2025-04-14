package frc.robot.utilities.files;

import java.io.FileWriter;
import java.nio.file.Files;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.Filesystem;

public class FileUtilities {
    public static boolean writeFile(String offsetPath, String content){
        try {
            FileWriter fileWriter = new FileWriter(Filesystem.getDeployDirectory() + "/" + offsetPath);
            fileWriter.write(content);
            fileWriter.close();

            return true;
        }catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    public static String readFile(String offsetPath){
        try {
            String fileRead = Files.readString(Paths.get(Filesystem.getDeployDirectory() + "/" + offsetPath));
            return fileRead;
        }catch (Exception e) {
            e.printStackTrace();
            return "";
        }
        
    }
}
