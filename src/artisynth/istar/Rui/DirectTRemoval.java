package artisynth.istar.Rui;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;

import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

public class DirectTRemoval {  
   
   public static ArrayList<Face> ctrFacesToRemove = new ArrayList<Face>();
   public static ArrayList<Face> ctrFacesToKeep = new ArrayList<Face>();
   ArrayList<ArrayList<Face>> allFacesToRemove = new ArrayList<ArrayList<Face>>();
   public static ArrayList<Vertex3d> ctrVertsToRemove = new ArrayList<Vertex3d>();
   public static ArrayList<Vertex3d> ctrVertsToKeep = new ArrayList<Vertex3d>();
   public static int numF =0;
  
   public static void removeTeeth(ArrayList<ArrayList<Vertex3d>> ctrVerts, PolygonalMesh mesh) {  
            
      for(int i=0; i< ctrVerts.size ();i++) {
       //midpointSubdivide(mesh,3, 1);
         for(Face f : mesh.getFaces () )
            f.clearVisited ();
         for(Vertex3d v : mesh.getVertices ())
            v.clearVisited ();
         
         deleteTeeth(ctrVerts.get(i), mesh);
         mesh.clearColors ();
         numF = mesh.numFaces ()-ctrFacesToRemove.size ();
         fillHole (ctrVerts.get (i),mesh);      
      }
//      mesh.setVertexColoringEnabled ();
//      for(int i=0; i<ctrVertsToRemove.size (); i++) 
//         mesh.setColor (ctrVertsToRemove.get (i).getIndex (), Color.red);
//  
//      for(int i=0; i<ctrVertsToKeep.size (); i++) 
//         mesh.setColor (ctrVertsToKeep.get (i).getIndex (), Color.green);

//      if(proceed == true) {
//                  
         mesh.removeFaces (ctrFacesToRemove);
         mesh.removeDisconnectedVertices ();     
             
         
//        return true;
//      }
//      else
//         return false;
   }
    
   public static boolean deleteTeeth(ArrayList<Vertex3d> ctrVerts, PolygonalMesh mesh ) {      
      ArrayList<Vertex3d> pntsToRemove = new ArrayList<Vertex3d>();
      ArrayList<Vertex3d> pntsToKeep = new ArrayList<Vertex3d>();
      ArrayList<Vertex3d> pntsToRemoveTemp1 = new ArrayList<Vertex3d>();
      ArrayList<Vertex3d> pntsToRemoveTemp2 = new ArrayList<Vertex3d>();
      ArrayList<Vertex3d> pntsToKeepTemp1 = new ArrayList<Vertex3d>();
      ArrayList<Vertex3d> pntsToKeepTemp2 = new ArrayList<Vertex3d>();
           
      mesh.setVertexColoringEnabled ();
      double minY = 100;
      double minX =100;
      double maxX =-100;
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
      
      for(int i = 0; i<ctrVerts.size () -minXId +1; i++) {
         Vertex3d vLast = ctrVerts.get (ctrVerts.size ()-1);
         ctrVerts.remove (ctrVerts.size ()-1);
         ctrVerts.add (0, vLast);
      }
      
      Vertex3d v0 = ctrVerts.get (0);
      Vertex3d v1 = ctrVerts.get (ctrVerts.size()/4);
      Vertex3d v2 = ctrVerts.get (3*ctrVerts.size()/4);
      
      Vector3d vec1 = new Vector3d();
      vec1.sub (v1.pnt, v0.pnt);
      Vector3d vec2 = new Vector3d();
      vec2.sub (v2.pnt, v0.pnt);     
      Vector3d crossP = vec1.cross (vec2);
      
      System.out.println(crossP);
      if(crossP.z >0) {
         System.out.println("clockwise==false");
         Collections.reverse (ctrVerts);         
      }
      else
         System.out.println("clockwise==true");
                         
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
         
         vStart= postVert;
         vEnd = preVert;  
                  
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
               if(ctrVertsToRemove.contains (v)==false 
               && ctrVertsToKeep.contains (v) ==false) {
                  ctrVertsToRemove.add (v);
                  pntsToRemove.add (v);
                  v.setVisited ();
               }
            }
            
            if(heTemp.getNext ().getFace ().isVisited () ==false) {
               if(ctrFacesToRemove.contains (heTemp.getNext ().getFace ())==false
               && ctrFacesToKeep.contains (heTemp.getNext ().getFace ())==false) {
                  ctrFacesToRemove.add (heTemp.getNext ().getFace ());
                  heTemp.getNext ().getFace ().setVisited ();
               }
            }
            
            heTemp= heTemp.getNext ().opposite;  
            //System.out.println("v.getIndex () "+ v.getIndex ());
            if(heTemp.getTail () == vStart) {
               mesh.setColor (vert.getIndex (), Color.red);
               return false;
            }
         } while(heTemp.getTail () != vEnd); 
         
         do {
            Vertex3d v = heTemp.getNext ().getHead ();
            if(v.isVisited ()==false) {                    
               if(ctrVertsToKeep.contains (v)==false) {
                  ctrVertsToKeep.add (v);
                  v.setVisited ();            
                  pntsToKeep.add (v);
               }
               if(ctrVertsToRemove.contains (v)==true)
                  ctrVertsToRemove.remove (v);
            }
            if(heTemp.getNext ().getFace ().isVisited () ==false) { 
               if(ctrFacesToKeep.contains (heTemp.getNext ().getFace ())==false) {
                  ctrFacesToKeep.add (heTemp.getNext ().getFace ());
                  heTemp.getNext ().getFace ().setVisited ();
               }
               if(ctrFacesToRemove.contains (heTemp.getNext ().getFace ())==true)
                  ctrFacesToRemove.remove (heTemp.getNext ().getFace ());
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
                  if(ctrFacesToKeep.contains (he2.getFace ())==false)
                     ctrFacesToKeep.add (he2.getFace ());
//                  if(ctrFacesToRemove.contains (he2.getFace ())==true)
//                     ctrFacesToRemove.remove (he2.getFace ());
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
                if(vTail.isVisited ()==false 
                && vTail.pnt.z >nearestCtrVrt.pnt.z // +5
                && vTail.pnt.y +2 > minY
                && vTail.pnt.x +3 > minX
                && vTail.pnt.x -3 <maxX) {                  
                   vTail.setVisited ();          
                   pntsToRemoveTemp2.add (vTail); 
                   if(ctrVertsToRemove.contains (vTail)==false)
                  // && ctrVertsToKeep.contains (vTail)==false)
                      ctrVertsToRemove.add (vTail);
                } 
                
                if(he2.getFace ().isVisited () ==false) {
                   if(ctrFacesToRemove.contains (he2.getFace ())==false)
                  // && ctrFacesToKeep.contains (he2.getFace ())==false)
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
             
        // if(vert0.pnt.x <vert1.pnt.x) {
            vert2 = ctrVerts.get (ctrVerts.size ()-2);
            ctrVerts.remove (vert1);          
            }
         else {          
            vert2 = ctrVerts.get(1);  
            ctrVerts.remove (vert0);
            }                
         mesh.addFace (vert0, vert1, vert2);         
       
      }while (ctrVerts.size ()>2);
   }

   public static void midpointSubdivide(PolygonalMesh mesh, double l, int iter) {
      
      for(int i=0; i<iter; i++) {              
         int numHE = 3*mesh.numFaces ();  
         numF= mesh.numFaces ();
         int numHE2 = numHE;
         for(int j = 0; j< numHE;j++) { 
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
               numHE2 += 6;
            }
         } 
         
         for(int j = 3*numF; j< numHE2;j++) { 
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
               numHE2 += 6;
            }
         }
      }           
   }     
}