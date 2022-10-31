import java.text.*;

public class testStrDouble {

    public static void main(String[] args) {
        // Prints "Hello, World" to the terminal window.
        
	    String	var = "bob";        // eg. q1, for joint var types
		String	symconst = "90";   // eg. L1, for lengths
		double	constant = 90; 

		System.out.println( var + symconst);
		System.out.println( var + constant);
		System.out.println( var + String.format("%f", constant));
		System.out.println( var + String.format("%g", constant));

		DecimalFormat format = new DecimalFormat("0.###");
		System.out.println( var + format.format(constant));
		constant = 1.234; 
		System.out.println( var + format.format(constant));

    }

}
