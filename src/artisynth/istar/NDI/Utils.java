package artisynth.istar.NDI;

public class Utils {
 
   /**
    * Converts the input byte to a string in hexadecimal, padded with '0's to
    * be 2 characters wide.
    *
    * @param input The integer to convert
    * @returns The hexadecimal representation of the integer as a String
    */
   public static String byteToHexString (byte input) {
      String str = Integer.toHexString (0xff & input);
      if (str.length() == 1) {
         str = "0" + str;
      }
      return str;
   }

   /**
    * Converts the input short to a string in hexadecimal, padded with '0's
    * to be 4 characters wide.
    *
    * @param input The integer to convert
    * @returns The hexadecimal representation of the integer as a String
    */
   public static String shortToHexString (short input) {
      String str = Integer.toHexString (0xffff & input);
      while (str.length() < 4) {
         str = "0" + str;
      }
      return str;
   }


   /**
    * Converts the input integer to a string in hexadecimal, padded with '0's
    * to be 8 characters wide.
    *
    * @param input The integer to convert
    * @returns The hexadecimal representation of the integer as a String
    */
   public static String intToHexString (int input) {
      return intToHexString (input, 8);
   }

   /**
    * Converts the input integer to a string in hexadecimal, padded with '0's
    * to be {@code width} characters wide.
    * 
    * @param input The integer to convert
    * @param width The width of the string, padded with zeros
    * @returns The hexadecimal representation of the integer as a String
    */
   public static String intToHexString(int input, int width) {
      StringBuilder sbuf = new StringBuilder();
      sbuf.append (Integer.toHexString (input));
      System.out.println ("sbuf=" + sbuf.toString() + " " + width);
      while (sbuf.length() < width) {
         sbuf.insert (0, '0');
      }
      return sbuf.toString();
   }

   /**
    * Converts the input integer to a string in decimal
    * @param input The integer to convert
    * @param width The width of the string, padded with zeros
    * @returns The decimal representation of the integer as a String
    */ 
   public static String intToString(int input, int width) {
      StringBuilder sbuf = new StringBuilder();
      if (input < 0) {
         sbuf.append (Integer.toString (-input));
         while (sbuf.length() < width-1) {
            sbuf.insert (0, '0');
         }
         sbuf.insert (0, '-');
      }
      else {
         sbuf.append (Integer.toString (input));
         while (sbuf.length() < width) {
            sbuf.insert (0, '0');
         }
      }
      return sbuf.toString();
   }

   /**
    * Converts a hex input string to an integer
    * @param input A string containing a hexadecimal number to convert to its integer equivalent.
    * @returns The integer representation of the input.
    */
   public static int hexStringToInt(String input) {
      return Integer.parseInt (input, 16);
   }

   /**
    * Converts a decimal input string to an integer
    * @param input A string containing a decimal number to convert to its integer equivalent.
    * @returns The integer representation of the input.
    */
   public static int stringToInt(String input) {
      return Integer.parseInt (input, 10);
   }


}
