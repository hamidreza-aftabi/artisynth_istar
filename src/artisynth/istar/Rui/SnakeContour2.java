//Jung, M., & Kim, H. (2004). Snaking across 3D meshes. 
//Paper presented at the 87-93. doi:10.1109/PCCGA.2004.1348338
package artisynth.istar.Rui;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;

import artisynth.core.mechmodels.RigidBody;
import maspack.collision.IntersectionContour;
import maspack.collision.IntersectionPoint;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SVDecomposition;
import maspack.matrix.Vector3d;

public class SnakeContour2 {    
   public static final double coneAngle = 180;
   public static final int numRays = 120;
   public final static int numProcessors = Runtime.getRuntime().availableProcessors();
   private static final double INF = Double.POSITIVE_INFINITY;
     
   public static Point3d midPnt = new Point3d( -2, 5, 0);
   
   public static void projection(
      ArrayList<IntersectionContour> contours,
      ArrayList<ArrayList<Vertex3d>> keyVerts,
      PolygonalMesh targetMesh,
      PolygonalMesh clipMesh,
      int pntNum) {
      
      int itv =0;
      for(int i=0; i<contours.size (); i++) {
         if(contours.get (i).size ()>50)
            itv+=contours.get (i).size (); 
      }
      itv /= pntNum;     //40 
      
      //System.out.println("itv "+ itv);
      for(int i=0; i<contours.size (); i++) {
         if(contours.get (i).size ()>3*itv-1) {           
            ArrayList<Vertex3d> keyTemp = new ArrayList<Vertex3d>();          
            for(int j =0; j<contours.get (i).size ()/itv; j++) {
               IntersectionPoint intersectPnt = new IntersectionPoint();
               if (contours.get (i).size ()==itv* j)
                  intersectPnt = contours.get (i).get (itv* j-1);
               else
                  intersectPnt = contours.get (i).get (itv* j);
               
              // System.out.println("num "+ itv* j);
               if(keyTemp.size ()>0 &&
               intersectPnt.distance (keyTemp.get (keyTemp.size ()-1).pnt)>12) {
                  intersectPnt = contours.get (i).get (itv*j -itv/2);
                  //System.out.println("numvvv "+ (itv*j -itv/2));
                  j--;
                  if(intersectPnt.distance (keyTemp.get (keyTemp.size ()-1).pnt)>10) {
                     intersectPnt = contours.get (i).get (itv*j + itv/4);
                     itv /=2;
                     j *=2;
                  }               
               }

               Vertex3d vi0 = intersectPnt.findSegmentFace (targetMesh).getVertex (0);
               Vertex3d vi1 = intersectPnt.findSegmentFace (targetMesh).getVertex (1);
               Vertex3d vi2 = intersectPnt.findSegmentFace (targetMesh).getVertex (2);         
               double di0 = vi0.distance (intersectPnt);
               double di1 = vi1.distance (intersectPnt);
               double di2 = vi2.distance (intersectPnt);

               if(di0== Math.min(di0, Math.min(di1, di2)))                                            
                  keyTemp.add (vi0);  
               if(di1== Math.min(di0, Math.min(di1, di2)))                                            
                  keyTemp.add (vi1); 
               if(di2== Math.min(di0, Math.min(di1, di2)))                                            
                  keyTemp.add (vi2);                     
            }
            keyVerts.add (keyTemp);    
               
         }      
      }            
   } 
   
   public static double planeFitting(
      ArrayList<Vertex3d> keyV, 
      PolygonalMesh mesh1, 
      PolygonalMesh mesh2) {  //teeth, original
      
      int verticesNum = mesh1.numVertices ();      
      ArrayList<GeoVertex> geoVertices = new ArrayList<GeoVertex>(); 
      MinCostComparator costComparator = new MinCostComparator();
      Vector3d baseV = new Vector3d(-1, 0,0);
      
      double minY = 100;
      double minX = 100;
      double maxX = -100;
            
      for(int i =0; i< verticesNum; i++) {
         Vertex3d v = mesh1.getVertex (i);
         if(v.pnt.y> 0)
            geoVertices.add (new GeoVertex(v, baseV.angle (new Vector3d(v.pnt.x, v.pnt.y,0))));         
         if(v.pnt.y <minY)
            minY = v.pnt.y;
         if(v.pnt.x <minX)
            minX = v.pnt.x;
         if(v.pnt.x >maxX)
            maxX = v.pnt.x;
         
//         midPnt.x += v.pnt.x;
//         midPnt.y += v.pnt.y;
//         midPnt.z += v.pnt.z;
      }            
      Collections.sort(geoVertices, costComparator); // ascending
      
//      midPnt.x /= verticesNum;
//      midPnt.y /= verticesNum;
//      midPnt.z /= verticesNum;      
//      System.out.println("midx "+ midPnt.x); 
//      System.out.println("midy "+ midPnt.y); 
      
      ArrayList <Integer> idxList = new ArrayList <Integer> ();
      int numP =5;
      for(int i =0; i<numP+1; i++)
         idxList.add (i*geoVertices.size ()/numP);
      
      double aveEdgeLength = mesh2.computeAverageEdgeLength ();
      
      for(int i = 0; i<numP; i++) {
      
         Point3d centerPnt = new Point3d (0,0,0);
         int vSize = idxList.get (i+1)- idxList.get (i);        
         double minAng = geoVertices.get (idxList.get (i)).getCost ();
         double maxAng = geoVertices.get (idxList.get (i+1)-1).getCost ();
         
         for(int j =idxList.get (i); j< idxList.get (i+1); j++) {
            Vertex3d v = geoVertices.get (j).getVert ();  
            centerPnt.x += v.pnt.x;
            centerPnt.y += v.pnt.y;
            centerPnt.z += v.pnt.z;
         }
         
         centerPnt.x /= vSize;
         centerPnt.y /= vSize;
         centerPnt.z /= vSize;
         MatrixNd A = new MatrixNd (3, vSize);
         
         for(int j =idxList.get (i); j< idxList.get (i+1); j++) {
            Vertex3d v = geoVertices.get (j).getVert ();
            Vector3d s = new Vector3d();
            A.setColumn (j-idxList.get (i), s.sub (v.pnt, centerPnt));
         }
        
         SVDecomposition svd = new SVDecomposition();
         svd.factor (A);
         MatrixNd U = svd.getU ();         
         Vector3d pNormal = new Vector3d();
         U.getColumn (2, pNormal);         
         Plane p = new Plane(pNormal, centerPnt); 
                           
         for(Vertex3d vert: mesh2.getVertices ()) {
            if(vert.pnt.y > minY- 2*aveEdgeLength && vert.pnt.x > minX-0.001  && vert.pnt.x < maxX+0.001) {
               double vertAng = baseV.angle (new Vector3d(vert.pnt.x, vert.pnt.y,0)); 
               if(i==0 || i== numP-1) {
                  if(vertAng <maxAng+0.1 && vertAng > minAng-0.1) {  
                     double distToP = p.distance (vert.pnt);
                     //System.out.println("distToP "+ distToP); 
                     if(Math.abs (distToP)< aveEdgeLength/3)  
                        keyV.add (vert);
                     }
                  }
               else if(vertAng <maxAng+0.1 && vertAng > minAng-0.1) { 
                  double distToP = p.distance (vert.pnt);
                  if(Math.abs (distToP)< aveEdgeLength/3) 
                     keyV.add (vert);
                  } 
               }
            }
//       RigidBody base = RigidBody.createBox ("base" +i, 20, 20, 0.1, 0.2);
//       RigidTransform3d T = new RigidTransform3d (centerPnt.x, centerPnt.y, centerPnt.z);
//       RotationMatrix3d R = new RotationMatrix3d();
//       R = ShapeDiameterFunction.computeRotation(new Vector3d (0, 0, 1), pNormal);
//       T.mulRotation (R);      
//       base.setPose (T);         
//       base.setDynamic (false);
//       mech.addRigidBody (base);
//       RenderProps.setAlpha(base, 1);  
      }
      mesh2.setVertexColoringEnabled();      
      for(int k = 0; k<keyV.size (); k++) 
         mesh2.setColor(keyV.get(k).getIndex (),Color.yellow);   

      return minY;
             
   }
     
   public static void vertsScreening(
      ArrayList<Vertex3d> keyV, 
      PolygonalMesh mesh,    
      double minY) {  //original, teethTeller
      
      for(int i =0; i< keyV.size (); i++) {
         Vertex3d vert = keyV.get (i);
         if(checkNormal(vert, mesh,  minY)== false) {
            keyV.remove (i);
            i--;
         }        
       }    
      double aveEdgeLength = mesh.computeAverageEdgeLength ();
      //System.out.println("aveEdgeLength "+ aveEdgeLength);
      for(int i =0; i<keyV.size ()-2; i++) {         
         for(int j = i+2; j<keyV.size()-1; j++) {  
            double penaltyi = keyV.get (i).distance (keyV.get (i+1));
            double penaltyj = keyV.get (i).distance (keyV.get (j));           
           
            Vector3d preNrm = new Vector3d();
            keyV.get (i).computeNormal (preNrm);
            Vector3d postNrmi = new Vector3d();
            keyV.get (i+1).computeNormal (postNrmi);
            Vector3d postNrmj = new Vector3d();
            keyV.get (j).computeNormal (postNrmj);
    
            double angi = 10*preNrm.angle (postNrmi);
            double angj = 10*preNrm.angle (postNrmj);
            
         //   System.out.println("angi "+ angi +" " +penaltyi);
         //   System.out.println("angj "+ angj +" " +penaltyj);
            penaltyi += angi;
            penaltyj += angj;              
            
            if(penaltyi > penaltyj)                      
               Collections.swap(keyV, i+1, j);           
            }
         
         if(keyV.get (i).distance (keyV.get (i+1)) < 5*aveEdgeLength/4) {
            keyV.remove (i+1);
            i--;
            }                                            
      }
      
      for(int i = 0; i < keyV.size (); i++) 
         mesh.setColor(keyV.get (i).getIndex (),Color.cyan);
      
   }
   public static boolean checkNormal(Vertex3d vert, PolygonalMesh mesh, double minY) {
            
      Vector3d zDir = new Vector3d(0,0,1);
      //System.out.println("zDir "+ zDir.x+" " + zDir.y+" " + zDir.z);
      Vector3d nrm = new Vector3d();
      vert.computeNormal (nrm);
      double angz = 180 * nrm.angle (zDir)/Math.PI;
       
      //discard points facing upward
      if(angz <45 && angz>120) 
         return false;
      
      if(vert.pnt.y < minY+5) //5
         return true;
      
      Vector3d nrmV = new Vector3d();
      vert.computeNormal (nrmV);
      nrmV.z =0;
      nrmV.normalize ();
      Vector3d nrmR = new Vector3d();
      nrmR.sub (vert.pnt, midPnt);
      nrmR.z = 0;
      nrmR.normalize ();
      double angR = 180 * nrmV.angle (nrmR)/Math.PI;
      
      if(angR>60 && angR <120)
         return false;
         
      
      Point3d vnear = new Point3d(vert.pnt.x +0.01*nrmV.x, vert.pnt.y +0.01*nrmV.y, vert.pnt.z +0.01*nrmV.z );
      BVFeatureQuery query = new BVFeatureQuery(); 
      Point3d pos = new Point3d();
      Face faceHit;
      faceHit = query.nearestFaceAlongRay (pos, null, mesh, vnear, nrmV);
      
      if(faceHit != null && vnear.distance (pos) <5 )
         return false;
      else
         return true;      
      
   }
      
   public static void checkSDF(ArrayList<Vertex3d> ctr, PolygonalMesh mesh,  double minY) {  
      ArrayList<Vertex3d> ctr1 = new ArrayList<Vertex3d> (ctr);
      for(int iter =0; iter<2;iter++) {
         for(int i = 0; i < ctr1.size (); i++) {
            Vertex3d vert = ctr1.get (i);
            double sdf = ShapeDiameterFunction.computeShapeDiameter(vert, mesh, coneAngle, numRays);
            vert  =  iterSDF(mesh, vert, sdf);  
            ctr1.set (i, vert);            
         }     
      } 
      
      // prevent verts from moving to a position facing more downward
      for(int i = 0; i < ctr.size (); i++) {
         Vertex3d vert0 = ctr.get (i);
         Vector3d nrm0 = new Vector3d();
         vert0.computeNormal (nrm0);
         Vertex3d vert1 = ctr1.get (i);
         Vector3d nrm1 = new Vector3d();
         vert1.computeNormal (nrm1);
         Vector3d subN = new Vector3d();
         subN.sub (nrm1, nrm0);
         subN.normalize ();
         if(subN.z>0.01 && checkNormal(vert1, mesh, minY)== true && vert0.distance (vert1) <4)
            ctr.set (i, vert1);
         
      }
     
   }
   
   public static Vertex3d iterSDF(PolygonalMesh mesh, Vertex3d vtx, double vtxSDF) {   
      
      Vertex3d vtxTemp = vtx;
      double sdfTemp = vtxSDF;  
      
      Vector3d nrm = new Vector3d();
      vtx.computeNormal (nrm);
    
      Iterator<HalfEdge> heIterator = vtxTemp.getIncidentHalfEdges ();       
      while (heIterator.hasNext()) {
         HalfEdge he = heIterator.next();
         Vertex3d vtxTail = he.tail;  
         
         if( vtxTail.pnt.z < vtx.pnt.z) {                                               
            double neighborSDF =  ShapeDiameterFunction.computeShapeDiameter(          
               vtxTail, mesh, coneAngle, numRays); 

            if(neighborSDF > sdfTemp ) {                          
               sdfTemp = neighborSDF;
               vtxTemp = vtxTail;
            }         
         }   
      }       
   
      return vtxTemp;        
   } 
   
   public static ArrayList<Vertex3d> snakeCurv(
      ArrayList<Vertex3d> keyV, 
      ArrayList<Vertex3d> ctrV, 
      PolygonalMesh mesh,
      double minY) {      
     
      ctrV = new ArrayList<Vertex3d>();
      int unlinkCount =0;
      //double aveEdgeLength = mesh.computeAverageEdgeLength ();
      for(int i = 0; i < keyV.size (); i++) {
         
         if(i<0)
            i=0;          
         Vertex3d vert = keyV.get (i);
         Vertex3d preVert = new Vertex3d();
         Vertex3d postVert = new Vertex3d();

         if(i==0)
            preVert = keyV.get (keyV.size ()-1);
         else
            preVert = keyV.get (i-1);

         if(i == keyV.size ()-1)
            postVert = keyV.get (0);
         else 
            postVert = keyV.get (i+1);
         
         if(preVert.distance (vert) > 20) {
            keyV.remove (preVert);
            i=i-2;
            continue;
         }
         if(preVert.distance (vert)<0.1) {
            System.out.println("Log: pre and vert are close"+ i);
            keyV.remove (i);
            i--;
            continue;
         }

         if(preVert.distance(postVert)+ 1< postVert.distance (vert)) {
            System.out.println("Log: post closer to pre than to vert"+ i);
            keyV.remove (i); 
            i--;
            continue;
         }

         Vector3d order2 = new Vector3d();
         order2.add (preVert.getPosition (), postVert.getPosition ());
         order2.sub (vert.getPosition ());
         order2.sub (vert.getPosition ());  

         ArrayList<GeoVertex> geoList = 
         ParallelGeodesicDistance.computeGeodesicDistance (preVert, vert, mesh);  
         
         if(unlinkCount > 25)
            return null;

         /// if preVert and vert are not connected
         if(geoList == null) {
            System.out.println(" unlinked "+ i); 
            unlinkCount ++;
            
            if(i ==0) { 
               System.out.println("remove prevert "+ keyV.indexOf (preVert)); 
               mesh.setColor(preVert.getIndex (),Color.green);    
               keyV.remove (preVert);
               i=i-2;
            }
            else {
               ArrayList<GeoVertex> geoList2 = 
               ParallelGeodesicDistance.computeGeodesicDistance (vert, postVert, mesh);
            
               if(geoList2==null) { 
                  System.out.println("remove vert "+ keyV.indexOf (vert));
                  mesh.setColor(preVert.getIndex (),Color.yellow); 
                  keyV.remove (vert);
                  i--;
                  }
               }
            continue;                          
         }

         double geoDist;
         if(geoList.size ()==0) 
            geoDist =0;                          
         else
            geoDist = geoList.get (geoList.size ()-1).getCost ();
         
         double intE =geoDist + 2* order2.norm(); //2
         double extE = 10* computeVtxCurvature(vert, mesh);
         double energy = intE +extE;
         ArrayList<GeoVertex> geoPath = geoList;

         Iterator<HalfEdge> heIterator = vert.getIncidentHalfEdges ();              
         while (heIterator.hasNext()) {
            HalfEdge he = heIterator.next();
            Vertex3d vtxTail = he.tail; 
            
           // prevent verts from moving to a position facing more downward         
            Vector3d nrm0 = new Vector3d();
            vert.computeNormal (nrm0);
            Vector3d nrm1 = new Vector3d();
            vtxTail.computeNormal (nrm1);
            Vector3d subN = new Vector3d();
            subN.sub (nrm1, nrm0);
            subN.normalize ();
            
            if(subN.z>0.01 && checkNormal(vtxTail, mesh, minY)== true) { // && postVert.distance (vtxTail)>1) {
               Vector3d order2N = new Vector3d();
               order2N.add (preVert.getPosition (), postVert.getPosition ());
               order2N.sub (vtxTail.getPosition ());
               order2N.sub (vtxTail.getPosition ()); 

               ArrayList<GeoVertex> geoListTemp = 
               ParallelGeodesicDistance.computeGeodesicDistance (preVert, vtxTail, mesh);
               double geoDistTemp;
              
               if(geoListTemp.size ()==0) {
                  geoDistTemp =0;
               }
               else
                  geoDistTemp = geoListTemp.get (geoListTemp.size ()-1).getCost ();

               double intEn = geoDistTemp + 2* order2N.norm(); //2
               double extEn = 10* computeVtxCurvature(vtxTail, mesh);
               double energyN = intEn +extEn;

               if(energyN < energy) {
                  energy = energyN;
                  vert = vtxTail; 
                  geoPath = geoListTemp;
               }  
            }
         }            
         for(int j =0; j< geoPath.size(); j++) 
            ctrV.add(geoPath.get(j).getVert());

         keyV.set (i, vert);            
      }

      //System.out.println("ctrV.size () " + ctrV.size ());
      ArrayList<GeoVertex> geoListTail = 
      ParallelGeodesicDistance.computeGeodesicDistance (ctrV.get (ctrV.size ()-1), ctrV.get (0), mesh);

      if(geoListTail!=null)
         for(int k =0; k< geoListTail.size(); k++) 
            ctrV.add(geoListTail.get(k).getVert()); 
      
      return ctrV;
   }
     
   public static ArrayList<Vertex3d> snakeVerts(Vertex3d v1, Vertex3d v2, PolygonalMesh mesh){
      
      ArrayList<Vertex3d> vPath = new ArrayList<Vertex3d>();
      Vertex3d tempV = v1;
      Vertex3d nextV = new Vertex3d();
      do {
         double dist0= 1000;          
         Iterator<HalfEdge> heIterator = tempV.getIncidentHalfEdges ();       
         while (heIterator.hasNext()) {
            HalfEdge he = heIterator.next();
            Vertex3d vtxTail = he.tail; 
            double dist = v2.distance (vtxTail) + 0.3 *computeVtxCurvature(vtxTail, mesh);
            if(dist<dist0) {
              dist0 = dist;
              nextV= vtxTail;
              } 
            }
         vPath.add (nextV);
         tempV= nextV;
         if(vPath.size ()>500) {
            vPath.clear ();
            break;
         }
            
      } while(tempV != v2);
      
      System.out.println("vPatha "+ vPath.size ());
      if(vPath.size ()==0) {
         tempV = v1;
         do {
            double dist0= 1000;          
            Iterator<HalfEdge> heIterator = tempV.getIncidentHalfEdges ();       
            while (heIterator.hasNext()) {
               HalfEdge he = heIterator.next();
               Vertex3d vtxTail = he.tail; 
               double dist = v2.distance (vtxTail);
               if(dist<dist0) {
                 dist0 = dist;
                 nextV= vtxTail;
                 } 
               }
            vPath.add (nextV);
            tempV= nextV;
            if(vPath.size ()>500) {
               vPath.clear ();
               break;
            }
            
         } while(tempV != v2);
         
         System.out.println("vPathb "+ vPath.size ());
         
            return vPath;
      }
      else
         return vPath;
   }
   
   public static void removeSmallCircles(ArrayList<Vertex3d> keyVerts, ArrayList<Vertex3d> ctrVerts, PolygonalMesh mesh) {      

      //remove small circles
      for(int i = 0; i < ctrVerts.size (); i++) {
         Vertex3d v1 = ctrVerts.get (i);
         for(int j = i+1; j < i+5; j++) {
            Vertex3d v2 = new Vertex3d();
            if(j> ctrVerts.size ()-1)
               v2= ctrVerts.get (j-ctrVerts.size ());
            else         
               v2 = ctrVerts.get (j);
            
            if(v1==v2) {
               System.out.println("resolve small circles");
               if(j< ctrVerts.size ())
                  for(int s=j; s>i;s--) {
                     if(keyVerts.contains (ctrVerts.get (s))&& ctrVerts.get (s) != v1) 
                        keyVerts.remove (ctrVerts.get (s));                   
                     ctrVerts.remove (s);
                  }
               else {
                  
                  int sizeTemp = ctrVerts.size ();
                  for(int s=ctrVerts.size ()-1; s>i;s--) {
                     if(keyVerts.contains (ctrVerts.get (s))&& ctrVerts.get (s) != v1) 
                        keyVerts.remove (ctrVerts.get (s)); 
                     ctrVerts.remove (s);
                  }
                  for(int t=j-sizeTemp; t>-1;t--) {
                     if(keyVerts.contains (ctrVerts.get (t))&& ctrVerts.get (t) != v1)                         
                        keyVerts.remove (ctrVerts.get (t));
                     ctrVerts.remove (t);
                   }
               }
            }
               
         }  
      }
               
      for(int i = 0; i < ctrVerts.size (); i++) 
         mesh.setColor(ctrVerts.get (i).getIndex (),Color.orange);  
     
   } 
   
   //https://computergraphics.stackexchange.com/questions/1718/
   //what-is-the-simplest-way-to-compute-principal-curvature-for-a-mesh-triangle     
    public static double computeVtxCurvature(Vertex3d vtx, PolygonalMesh mesh) {   
       Vector3d nrm1 = new Vector3d();
       vtx.computeNormal (nrm1);
       double vtxCurv = 0;
       int eCount = 0;
       Iterator<HalfEdge> heIterator = vtx.getIncidentHalfEdges ();  
       while (heIterator.hasNext()) {
          HalfEdge he = heIterator.next();
          Vertex3d vtxTail = he.tail;
          
       // mainly compute horizontal curvature
          Vector3d curvDir = new Vector3d();
          curvDir.sub (vtxTail.getPosition (), vtx.getPosition ());
          curvDir.normalize ();
          Vector3d zDir = new Vector3d(0,0,1);
          double ang = Math.min (180 * zDir.angle (curvDir)/Math.PI,
             180 - 180 * zDir.angle (curvDir)/Math.PI);
          if(ang<30)
             continue;
          
          eCount++;
          
          Vector3d nrm2 = new Vector3d();
          vtxTail.computeNormal (nrm2);
          
          Vector3d nrmSub = new Vector3d();
          nrmSub.sub (nrm2, nrm1);
          Vector3d pntSub = new Vector3d();
          pntSub.sub(vtxTail.pnt,vtx.pnt);
          double dist = vtx.distance (vtxTail);
          
          double curv = nrmSub.dot(pntSub)/Math.pow(dist, 2);
          vtxCurv+=curv;
          
    }
       vtxCurv/=eCount;
       return vtxCurv;
       
    }
      
}


