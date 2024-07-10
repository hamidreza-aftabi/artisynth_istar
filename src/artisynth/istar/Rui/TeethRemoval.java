package artisynth.istar.Rui;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Iterator;

import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

public class TeethRemoval {  
   
   public static ArrayList<Face> ctrFacesToRemove = new ArrayList<Face>();
   public static ArrayList<Face> ctrFacesToKeep = new ArrayList<Face>();
   public static ArrayList<Vertex3d> ctrVertsToRemove = new ArrayList<Vertex3d>();
   public static int numF =0;
   public static boolean clockwise = true;
   
   public static boolean removeTeeth(ArrayList<Vertex3d> ctrVerts, PolygonalMesh mesh) {     
      
      StopWatch timer = new StopWatch("timer");
      timer.checkpoint("Start removing teeth");
      
      boolean proceed = deleteTeeth(ctrVerts, mesh);
      
      if(proceed == true) {
       //  mesh.clearColors ();
         numF = mesh.numFaces ()-ctrFacesToRemove.size ();
      //   fillHole (ctrVerts,mesh);
         
         mesh.setVertexColoringEnabled ();
         for(int i=0; i< ctrVertsToRemove.size (); i++) {
            if(ctrVerts.contains (ctrVertsToRemove.get (i))== false)
               mesh.setColor (ctrVertsToRemove.get (i).getIndex (), new Color(0.8f, 0.8f, 1.0f));
         }
         
//         mesh.removeFaces (ctrFacesToRemove);
//         //mesh.removeVertices (ctrVertsToRemove);
//         mesh.removeDisconnectedVertices ();         
//         //midpointSubdivide(mesh,3, 1);    
         
         timer.checkpoint("Done!");
         System.out.println("ctrVertsToRemove "+ ctrVertsToRemove.size ()); 
         
        return true;
      }
      else
         return false;
   }
    
   public static boolean deleteTeeth(ArrayList<Vertex3d> ctrVerts, PolygonalMesh mesh ) {      
      ArrayList<Vertex3d> pntsToRemove = new ArrayList<Vertex3d>();
      ArrayList<Vertex3d> pntsToKeep = new ArrayList<Vertex3d>();
      ArrayList<Vertex3d> pntsToRemoveTemp1 = new ArrayList<Vertex3d>();
      ArrayList<Vertex3d> pntsToRemoveTemp2 = new ArrayList<Vertex3d>();
      ArrayList<Vertex3d> pntsToKeepTemp1 = new ArrayList<Vertex3d>();
      ArrayList<Vertex3d> pntsToKeepTemp2 = new ArrayList<Vertex3d>();
           
      double minY = 0;
      double minX =100;
      double maxX =0;
      double maxY = -100;
      int maxYId =0;
      int minXId =0;
      for(int j = 0; j < ctrVerts.size (); j++) {
         ctrVerts.get (j).setVisited ();          
         minY = Math.min (minY, ctrVerts.get (j).pnt.y);
         //minX = Math.min (minX, ctrVerts.get (j).pnt.x);
         maxX = Math.max (maxX, ctrVerts.get (j).pnt.x);
         if(ctrVerts.get (j).pnt.y >maxY) {
            maxY = ctrVerts.get (j).pnt.y;
            maxYId = j;
         }
         if(ctrVerts.get (j).pnt.x <minX) {
            minX = ctrVerts.get (j).pnt.x;
            minXId = j;
         }        
      }
      if(maxYId > ctrVerts.size ()/2) {
         clockwise = false;
         System.out.println("clockwise==false");
      }
      else
         System.out.println("clockwise==true");
      
      if(clockwise==true) {
         for(int i = 0; i<ctrVerts.size () -minXId +1; i++) {
            Vertex3d vLast = ctrVerts.get (ctrVerts.size ()-1);
            ctrVerts.remove (ctrVerts.size ()-1);
            ctrVerts.add (0, vLast);
         }
      }
      else {
         for(int i = 0; i<minXId +1; i++) {
            Vertex3d vFirst = ctrVerts.get (0);
            ctrVerts.remove (0);
            ctrVerts.add (vFirst);
         }
      }
         
           
      for(int i = 0; i < ctrVerts.size (); i++) {
         Vertex3d vert = ctrVerts.get (i);
         Vertex3d preVert = new Vertex3d();
         Vertex3d postVert = new Vertex3d();
         Vertex3d vStart = new Vertex3d();
         Vertex3d vEnd = new Vertex3d();
         
         if(i==0)
            preVert = ctrVerts.get (ctrVerts.size ()-1);
         else
            preVert = ctrVerts.get (i-1);
         
         if(i == ctrVerts.size ()-1)
            postVert = ctrVerts.get (0);
         else 
            postVert = ctrVerts.get (i+1);
         
         if(preVert==vert || postVert==vert || preVert==postVert) {
            System.out.println("remove pre/post "+ i);
            ctrVerts.remove (i);
            i--;
            continue;
         }
 
         if(clockwise==true) {
            vStart= postVert;
            vEnd = preVert;  
         }
         else {
            vStart= preVert;
            vEnd = postVert; 
         }
         
         HalfEdge heTemp = new HalfEdge();
         Iterator<HalfEdge> heIterator = vert.getIncidentHalfEdges ();       
         while (heIterator.hasNext()) {
            HalfEdge he = heIterator.next();
            Vertex3d vtxTail = he.tail;  
            if(vtxTail== vStart) {
               heTemp= he;    
               break;
            } 
         }
         if(heTemp.getTail () != vStart) {
            mesh.setColor (vert.getIndex (), Color.red);
           // System.out.println("heTemp.getTail ().getIndex () "+ heTemp.getTail ().getIndex ());
            return false;
         }
         do {
            Vertex3d v = heTemp.getNext ().getHead ();
            if(v.isVisited ()==false) {             
               pntsToRemove.add (v);
               ctrVertsToRemove.add (v);
               v.setVisited ();
            }
            
            if(heTemp.getNext ().getFace ().isVisited () ==false) {
               ctrFacesToRemove.add (heTemp.getNext ().getFace ());
               heTemp.getNext ().getFace ().setVisited ();
            }
          
            heTemp= heTemp.getNext ().opposite;  
            
            if(heTemp.getTail () == vStart) {
               mesh.setColor (vert.getIndex (), Color.red);
               return false;
            }
         } while(heTemp.getTail () != vEnd); 
         
         do {
            Vertex3d v = heTemp.getNext ().getHead ();
            if(v.isVisited ()==false) {       
               v.setVisited ();            
               pntsToKeep.add (v);
            }
            if(heTemp.getNext ().getFace ().isVisited () ==false) {  
               ctrFacesToKeep.add (heTemp.getNext ().getFace ());
               heTemp.getNext ().getFace ().setVisited ();
            }
            heTemp= heTemp.getNext ().opposite;    
            
         } while(heTemp.getTail () != vStart);                   
      }
      
         pntsToRemoveTemp1 = pntsToRemove;         
         pntsToKeepTemp1 = pntsToKeep;
          
      for(int iter =0; iter<10;iter++) {
         
         int pntsToKeepNum = pntsToKeepTemp1.size ();
         for(int i = 0; i < pntsToKeepNum; i++) {                   
            Vertex3d teethV = pntsToKeepTemp1.get (i);
            Iterator<HalfEdge> heIterator2 = teethV.getIncidentHalfEdges ();       
            while (heIterator2.hasNext()) {
               HalfEdge he2 = heIterator2.next();
               Vertex3d vTail = he2.tail;  
               if(vTail.isVisited ()==false) {          
                  vTail.setVisited ();                             
                  pntsToKeepTemp2.add (vTail);                
               } 

               if(he2.getFace ().isVisited () ==false) {  
                  ctrFacesToKeep.add (he2.getFace ());
                  he2.getFace ().setVisited ();
               }
            }
         }
         pntsToKeepTemp1 = pntsToKeepTemp2;
         pntsToKeepTemp2 = new ArrayList<Vertex3d>();
            
         int pntsToRemoveNum = pntsToRemoveTemp1.size ();
          for(int i = 0; i < pntsToRemoveNum; i++) {                   
             Vertex3d teethV = pntsToRemoveTemp1.get (i);
             Iterator<HalfEdge> heIterator2 = teethV.getIncidentHalfEdges ();       
             while (heIterator2.hasNext()) {
                HalfEdge he2 = heIterator2.next();
                Vertex3d vTail = he2.tail; 
                Vertex3d nearestCtrVrt = locatedNearestCtrVert(vTail, ctrVerts);                
                if(vTail.isVisited ()==false ) {
//                && vTail.pnt.z +0.5>nearestCtrVrt.pnt.z // +5
//                && vTail.pnt.y  > minY
//                && vTail.pnt.x  > minX
//                && vTail.pnt.x  <maxX) {                  
                   vTail.setVisited ();          
                   pntsToRemoveTemp2.add (vTail); 
                   ctrVertsToRemove.add (vTail);
                } 
                
                if(he2.getFace ().isVisited () ==false) {
                   ctrFacesToRemove.add (he2.getFace ());
                   he2.getFace ().setVisited ();
                }          
             }
          }          
          pntsToRemoveTemp1 = pntsToRemoveTemp2;
          if (pntsToRemoveTemp2.size ()<10)
             break;
          else
             pntsToRemoveTemp2 = new ArrayList<Vertex3d>();                 
      }  
      
      return true;
} 
    
   
   public static Vertex3d locatedNearestCtrVert(Vertex3d v, ArrayList<Vertex3d> keyVerts) {
      
      Vertex3d nearestVert = new Vertex3d();
      double minDist =1000;
      for (Vertex3d vert:keyVerts) {
         double dist = vert.distance (v);
         if(dist<minDist) {
            nearestVert = vert;
            minDist=dist;
         }
      }
      
      return nearestVert;
   }
   
   public static void fillHole(ArrayList<Vertex3d> ctrVerts, PolygonalMesh mesh ) { 
      Vertex3d vert0 = new Vertex3d();
      Vertex3d vert1 = new Vertex3d();
      Vertex3d vert2 = new Vertex3d();      
    
      do{ 
         vert0 = ctrVerts.get (0);
         vert1 = ctrVerts.get (ctrVerts.size ()-1);
         
         if(vert0.distance (ctrVerts.get (ctrVerts.size ()-2)) 
         < vert1.distance (ctrVerts.get(1))) {
             
         //if(vert0.pnt.x <vert1.pnt.x) {
            vert2 = ctrVerts.get (ctrVerts.size ()-2);
            ctrVerts.remove (vert1);          
            }
         else {          
            vert2 = ctrVerts.get(1);  
            ctrVerts.remove (vert0);
            }
         
         if(clockwise==true)
            mesh.addFace (vert0, vert1, vert2);
         else
            mesh.addFace (vert0, vert2, vert1);
       
      }while (ctrVerts.size ()>2);
      
//      for(int i = 0; i < ctrVerts.size ()-2; i++) { 
//         Vertex3d vert0 = new Vertex3d();
//         Vertex3d vert1 = new Vertex3d();
//         Vertex3d vert2 = new Vertex3d();
//         
//         if(i%2 == 0) {  
//            vert0 = ctrVerts.get (i/2);
//            vert1 = ctrVerts.get (ctrVerts.size ()-i/2-1);
//            vert2 = ctrVerts.get (i/2+1);         
//         }
//         else {
//            vert0 = ctrVerts.get ((i+1)/2);
//            vert1 = ctrVerts.get (ctrVerts.size ()-(i+1)/2);
//            vert2 = ctrVerts.get (ctrVerts.size ()-(i+1)/2-1);         
//         }
//         if(clockwise==true)
//            mesh.addFace (vert0, vert1, vert2);
//         else
//            mesh.addFace (vert0, vert2, vert1);
//   
//      }   
   }

   public static void midpointSubdivide(PolygonalMesh mesh, double l, int iter) {
      
      for(int i=0; i<iter; i++) {              
         int numHE = 3*mesh.numFaces ();         
         for(int j = 3*numF; j< numHE;j++) { 
     //       HalfEdge he = mesh.getFace (j).firstHalfEdge ();
     //       System.out.println("numHE "+ numHE);
     //       System.out.println("j "+ j);  
            HalfEdge he = mesh.getHalfEdge (j);
            Vertex3d vert = he.head;
            Vertex3d vTail = he.tail;
          
            if(he.length ()>l) {
               Vertex3d vn1 = he.getNext ().head;
               Vertex3d vn2 = he.opposite.getNext ().head; 

               Point3d pMid = new Point3d(0.5*(vert.pnt.x +vTail.pnt.x),
                  0.5*(vert.pnt.y +vTail.pnt.y),
                  0.5*(vert.pnt.z +vTail.pnt.z));

               Face f = he.getFace ();
               Face f0 = he.getOppositeFace ();
               Vertex3d vMid = new Vertex3d(pMid);
               mesh.addVertex (vMid);                  
               mesh.removeFace (f);
               mesh.removeFace (f0);
               mesh.addFace (vTail, vMid, vn1);
               mesh.addFace (vn1, vMid, vert);               
               mesh.addFace (vert, vMid, vn2);
               mesh.addFace (vn2, vMid, vTail);
               //numHE += 6;
            }
         }           
      }           
   }     
}