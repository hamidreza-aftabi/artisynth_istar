 package artisynth.istar.Rui;
import java.util.Comparator;

class MinCostComparator implements Comparator<GeoVertex> {  
   @Override
   public int compare(GeoVertex v1, GeoVertex v2) {
      double c1 = v1.getCost();
      double c2 = v2.getCost();
      if (c1 < c2) {
         return -1;
      } else if (c1 > c2) {
         return 1;
      }
      return 0;
   }      
}



