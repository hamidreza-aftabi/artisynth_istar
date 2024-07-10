/**
 * Copyright (c) 2020, by the Author: Rui Yang (UBC)
 */
package artisynth.istar.Rui;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.util.BinaryHeap;

/**
 * Geodesic distance for undirected graph
 */
public class ParallelGeodesicDistance {
   
   public final static int numProcessors = Runtime.getRuntime().availableProcessors();
   
   // Dijkstra's algorithm
   @SuppressWarnings("unchecked")
   public static ArrayList<GeoVertex> computeGeodesicDistance(
      Vertex3d vtxStart, 
      Vertex3d vtxEnd, 
      PolygonalMesh mesh) {
    
      ArrayList<GeoVertex>[] geoVertices = new ArrayList[numProcessors];
      BinaryHeap<GeoVertex>[] unvisited = new BinaryHeap[numProcessors];
      MinCostComparator costComparator = new MinCostComparator();
      
 //     System.out.println("numProcessors " + numProcessors);
//      StopWatch timer = new StopWatch("timer");
  //    timer.checkpoint("1");
      
      for(int i = 0; i< numProcessors; i++) {
         geoVertices[i] = new ArrayList<GeoVertex>();
         unvisited[i] = new BinaryHeap<GeoVertex>(costComparator);
      }
     
      int col = mesh.numVertices ()/numProcessors+1;     
      for (int i = 0; i < numProcessors; i++)
         for(int j =0; j < col; j++)
            if(i* col+j< mesh.numVertices ())
               geoVertices[i].add (new GeoVertex(mesh.getVertex (i*col +j), Double.MAX_VALUE));
           
      geoVertices[vtxStart.getIndex ()/col].get (vtxStart.getIndex ()%col).setCost (0);      
      
      for(int i = 0; i < numProcessors; i++)
         unvisited[i].set(geoVertices[i]);
 
      GeoVertex currLoc = new GeoVertex(vtxStart,0);
      ArrayList<Vertex3d> visited = new ArrayList<Vertex3d>();   
      
      while(visited.size ()<mesh.numVertices ()) {
         
         ExecutorService executor = Executors.newFixedThreadPool(numProcessors);
         List<Future<?>> futures = new ArrayList<Future<?>>();         
         int core =0;
         
         // set currLoc to be the vert with smallest cost value among unvisited verts
         int num=0;
         do {
            currLoc = unvisited[num].peek ();
            num++;
            }while(currLoc==null);
         
         for(int i = 0; i < numProcessors; i++) {           
            if(unvisited[i].size ()>0) {
               if(currLoc.getCost () > unvisited[i].peek().getCost ()) {
                   currLoc = unvisited[i].peek();
                   core =i;
               }
            }
         }
         unvisited[core].poll ();
         visited.add (currLoc.getVert ());
         
         if(currLoc.getCost ()==Double.MAX_VALUE || visited.size () > mesh.numVertices ())///4
            return null;
         if (currLoc.getVert () == vtxEnd) 
            break;
                  
         boolean[] costsChanged = new boolean [numProcessors];               
         Iterator<HalfEdge> heIterator = currLoc.getVert ().getIncidentHalfEdges ();              
         while (heIterator.hasNext()) {
            HalfEdge he = heIterator.next();
            Vertex3d vtxTail = he.tail;       
            
            if(visited.contains (vtxTail)==false) {            
               double altCost = currLoc.getCost () + he.length ();//currLoc.getVert ().distance (vtxTail);
               if(altCost < geoVertices[vtxTail.getIndex ()/col].get (vtxTail.getIndex ()%col).getCost ()) {
                  geoVertices[vtxTail.getIndex ()/col].get (vtxTail.getIndex ()%col).setCost (altCost);
                  geoVertices[vtxTail.getIndex ()/col].get (vtxTail.getIndex ()%col).setPreVert (currLoc);      
                  costsChanged[vtxTail.getIndex ()/col] = true;                  
               }
            }
         }
          
//         System.out.println("costsChanged ");
//         for(int s =0; s< costsChanged.length; s++)
//            System.out.println("costsChanged "+ costsChanged[s]);
        
         for(int i = 0; i < numProcessors; i++) {            
            if (costsChanged[i]) {
               final int t =i;
               Future<?> f = executor.submit(new Runnable() {
                  public void run() {
                     unvisited[t].update ();
                     }});
                 futures.add(f);
              }
            }
//         if(futures.size () >2)
//            System.out.println("futures.size () "+ futures.size ());
         
         // Await all runnables to be done (blocking)
         for(Future<?> future : futures) {
            try {
               future.get();  // get will block until the future is done
            }
            catch (InterruptedException e) {
               // TODO Auto-generated catch block
               e.printStackTrace();
            }
            catch (ExecutionException e) {
               // TODO Auto-generated catch block
               e.printStackTrace();
            }      
         }     
         executor.shutdown ();        
      }
          
      ArrayList<GeoVertex> path = new ArrayList<GeoVertex>();                 
      // vtxStart+1 ~ vtxEnd
      while (currLoc.getVert () != vtxStart) {
         path.add (0, currLoc);
         currLoc = currLoc.getPreVert ();    
      }

      return path;

   }
}
