package artisynth.istar.NDI;

public class Split {

   public static void main (String[] args) {
      String foo = "here  id=4   --77";
      for (String s : foo.split(" ")) {
         System.out.println ("'"+s+"'");
      }
   }
}
