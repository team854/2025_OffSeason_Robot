package frc.robot.utilities.text;

public class TextUtilities {
    public static String removeLowerCase(String str) {
        StringBuilder outputStr = new StringBuilder();

        for (char currentChar : str.toCharArray()) {
            if (!Character.isLowerCase(currentChar)){
                outputStr.append(currentChar);
            }
        }
        return outputStr.toString();
    }
}
