/**
 * Copyright (c) 2020, by the Author: Rui Yang (UBC)
 */
package artisynth.istar.Rui;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;

import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.util.BinaryHeap;

/**
 * Geodesic distance for undirected graph
 */
public class GeodesicDistance {
   
   private static class MinCostComparator implements Comparator<GeoVertex> {  
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

   // Dijkstra's algorithm
   public static ArrayList<GeoVertex> computeGeodesicDistance(
      Vertex3d vtxStart, 
      Vertex3d vtxEnd, 
      PolygonalMesh mesh) {
      
      StopWatch timer = new StopWatch("timer");
      timer.checkpoint("1");
      
      ArrayList<GeoVertex> geoVertices = new ArrayList<GeoVertex>();
      for (int i = 0; i < mesh.numVertices (); i++)
         geoVertices.add (new GeoVertex(mesh.getVertex (i), Double.MAX_VALUE));
        
      geoVertices.get (vtxStart.getIndex ()).setCost (0);     
      MinCostComparator costComparator = new MinCostComparator();
      BinaryHeap<GeoVertex> unvisited = new BinaryHeap<GeoVertex>(costComparator);
      unvisited.set(geoVertices);
      GeoVertex currLoc = new GeoVertex(vtxStart,0);
      ArrayList<Vertex3d> visited = new ArrayList<Vertex3d>();
      
      timer.checkpoint("2");
      
      while (unvisited.size() > 0) {

         // find vertex with smallest cost
         currLoc = unvisited.poll(); 
         visited.add (currLoc.getVert ());
         
         if(currLoc.getCost ()==Double.MAX_VALUE)// || visited.size ()>7000 )
            return null;
         if (currLoc.getVert () == vtxEnd) 
            break;
         
         boolean costsChanged = false;
         Iterator<HalfEdge> heIterator = currLoc.getVert ().getIncidentHalfEdges ();              
         while (heIterator.hasNext()) {
            HalfEdge he = heIterator.next();
            Vertex3d vtxTail = he.tail;            
               
            if(visited.contains (vtxTail)==false) {
            
               //System.out.println("vTail "+ vtxTail.getIndex ());
               double altCost = currLoc.getCost () + he.length ();//currLoc.getVert ().distance (vtxTail);
               if(altCost < geoVertices.get (vtxTail.getIndex ()).getCost ()) {
                  geoVertices.get (vtxTail.getIndex ()).setCost (altCost);
                  geoVertices.get (vtxTail.getIndex ()).setPreVert (currLoc); 
      
                  costsChanged = true;
               }
            }
         }
                  
         if (costsChanged)
            unvisited.update();         
      }
      
      timer.checkpoint("3");
     // System.out.println("visited "+ visited.size ());
      
         //double dist = currLoc.getCost ();         
         ArrayList<GeoVertex> path = new ArrayList<GeoVertex>();
         
         
         // vtxStart+1 ~ vtxEnd
         while (currLoc.getVert () != vtxStart) {
            path.add (0, currLoc);
            currLoc = currLoc.getPreVert ();    
         }
         
         return path;
         
      }
}
