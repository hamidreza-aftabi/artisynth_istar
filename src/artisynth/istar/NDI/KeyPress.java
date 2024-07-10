package artisynth.istar.NDI;

import java.io.*;

public class KeyPress {

   public static void main (String[] args) {
      while (true) {
         try {
            System.out.println (System.in.available());
            Thread.sleep (1000);
         }
         catch (Exception e) {
            e.printStackTrace(); 
         }
      }
      
   }
}
