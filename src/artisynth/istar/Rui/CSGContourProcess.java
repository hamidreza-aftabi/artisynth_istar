//Jung, M., & Kim, H. (2004). Snaking across 3D meshes. 
//Paper presented at the 87-93. doi:10.1109/PCCGA.2004.1348338
package artisynth.istar.Rui;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import artisynth.core.mechmodels.Point;
import maspack.collision.IntersectionContour;
import maspack.collision.IntersectionPoint;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

public class CSGContourProcess {  

   static ArrayList<Vertex3d> verToKeep = new ArrayList<Vertex3d>(); 
   public static final double coneAngle = 150;
   public static final int numRays = 50;
   public final static int numProcessors = Runtime.getRuntime().availableProcessors();
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
   
   public static void gaussianDeform(
      ArrayList<ArrayList<Vertex3d>> keyVerts,    
      PolygonalMesh mesh){
     
      double var = 10;          
      ArrayList<Vertex3d> keyV = new ArrayList<Vertex3d>();
      for(int i=0; i<keyVerts.size ();i++) {
         for(int j=0; j<keyVerts.get (i).size ();j++) {
            keyV.add (keyVerts.get (i).get (j));
         }}
      @SuppressWarnings("unchecked")
      ArrayList<GeoVertex>[] geoRange = new ArrayList[keyV.size ()];
      for (int i = 0; i < keyV.size (); i++) 
         geoRange[i] = new ArrayList<GeoVertex>(); 
     
      for(int i=0; i<mesh.numVertices ();i++) { 
         Vertex3d v =mesh.getVertex (i);
         int ctrlVertNum = -1;
         double minDist = 100;
         for(int j=0; j<keyV.size ();j++) {            
            Vertex3d vTemp = keyV.get (j);           
            double dist = v.distance (vTemp); 
            if(dist<minDist) { 
               minDist=dist;
               ctrlVertNum =j;
            }
         }
         if(minDist<12) {
            GeoVertex vd = new GeoVertex(v, minDist);
            geoRange[ctrlVertNum].add(vd);
         }         
      }  
 
      for(int i=0; i<keyV.size ();i++) {  
         Vertex3d vert = keyV.get (i);
         if(geoRange[i].size ()!=0) {
            MinCostComparator costComparator = new MinCostComparator();
            Collections.sort(geoRange[i], costComparator); 
    
            Vertex3d v0 = geoRange[i].get (0).getVert ();
            Vector3d diff = new Vector3d();
            diff.sub (vert.pnt, v0.pnt);  
            double dist = geoRange[i].get (0).getCost ();          
            double par = (1/(var * Math.sqrt (2*Math.PI)))
               * Math.exp (-0.5* (dist/var)*(dist/var));
            v0.getPosition ().add (diff);
                                
            for(int j=1; j<geoRange[i].size ();j++) {       
               Vertex3d neighbor = geoRange[i].get (j).getVert();
               double dist1 = neighbor.distance (vert);
               double par1 = (1/(var * Math.sqrt (2*Math.PI))) 
                  * Math.exp (-0.5* (dist1/var)*(dist1/var))/par;                 
               neighbor.getPosition ().scaledAdd (par1, diff); 
             }
         }
      }       
   }
   
   public static void gaussianManualDeformSingleP(
      Point3d tr,
      Point selectedPnt,  
      ArrayList<GeoVertex> vertDist,
      ArrayList<Point> controlPoints,
      PolygonalMesh target,
      double radius,
      boolean isOneSideCtrl,
      boolean isClipToSurface){
      
      BVFeatureQuery query = new BVFeatureQuery();  
      if(isClipToSurface==true) {
         Point3d nearPnt = new Point3d();
         query.nearestFaceToPoint (nearPnt, null, target, selectedPnt.getPosition ());
         selectedPnt.setPosition (nearPnt);
      }
    
      double var = radius/2;
      Vertex3d vert0 = vertDist.get (0).getVert ();   
      double dist = vert0.distance (selectedPnt.getPosition ()); 
      double par = (1/(var * Math.sqrt (2*Math.PI))) 
         * Math.exp (-0.5* (dist/var)*(dist/var));
      vert0.getPosition ().add (tr); 
     
      Vertex3d nearV0 = new Vertex3d();
      nearV0=query.nearestVertexToPoint (target, vert0.pnt);
      Vector3d nrm0= new Vector3d();
      nearV0.computeNormal (nrm0);
      
      for(int i=1; i<vertDist.size (); i++) {
         Vertex3d vert = vertDist.get (i).getVert (); 
         double dist1 = vertDist.get (i).getCost ();
         if(dist1< radius) {            
            if(isOneSideCtrl==false) {
               double par1 = (1/(var * Math.sqrt (2*Math.PI))) 
               * Math.exp (-0.5* (dist1/var)*(dist1/var))/par;               
               vert.getPosition ().scaledAdd (par1, tr); 
               }
            else {
               Vertex3d nearV1 = new Vertex3d();
               nearV1=query.nearestVertexToPoint (target, vert.pnt);
               Vector3d nrm1= new Vector3d();
               nearV1.computeNormal (nrm1);
               double ang = 180 * nrm0.angle (nrm1)/Math.PI;
               if(ang<90) {
                  double par1 = (1/(var * Math.sqrt (2*Math.PI))) 
                  * Math.exp (-0.5* (dist1/var)*(dist1/var))/par;                                
                  vert.getPosition ().scaledAdd (par1, tr); 
               }
            }
         } 
         else
            break;
      }    
    
      for(int i=0; i<controlPoints.size ();i++) {  
         Point p = controlPoints.get (i);
         double distP = selectedPnt.distance (p);
         if(p!= selectedPnt && distP<radius) {         
            if(isOneSideCtrl==false) {
               double par1 = (1/(var * Math.sqrt (2*Math.PI))) 
               * Math.exp (-0.5* (distP/var)*(distP/var))/par;                
               p.getPosition ().scaledAdd (par1, tr);                
               if(isClipToSurface==true){
                  Point3d nearPnt = new Point3d();
                  query.nearestFaceToPoint (nearPnt, null, target, p.getPosition ());
                  p.setPosition (nearPnt);
               }                      
            }
            else {
               Vertex3d nearV1 = new Vertex3d();
               nearV1=query.nearestVertexToPoint (target, p.getPosition ());
               Vector3d nrm1= new Vector3d();
               nearV1.computeNormal (nrm1);
               double ang = 180 * nrm0.angle (nrm1)/Math.PI;
               if(ang<90) {
                  double par1 = (1/(var * Math.sqrt (2*Math.PI))) 
                  * Math.exp (-0.5* (distP/var)*(distP/var))/par;                
                  p.getPosition ().scaledAdd (par1, tr);                               
                  if(isClipToSurface==true) {
                     Point3d nearPnt = new Point3d();
                     query.nearestFaceToPoint (nearPnt, null, target, p.getPosition ());
                     p.setPosition (nearPnt);
                  }             
               }               
            }
         }               
      }
   }
   
   public static void gaussianManualDeformMultiP(
      Point3d tr,
      ArrayList<Point> pList, 
      ArrayList<ArrayList<GeoVertex>> vertDistList,
      PolygonalMesh target,
      boolean isClipToSurface){
    
      double var = 5;     
      BVFeatureQuery query = new BVFeatureQuery();  
      
      for(int i=0; i<vertDistList.size (); i++) {  
         Point p =pList.get (i);     
         if(isClipToSurface==true) {
            Point3d nearPnt = new Point3d();
            query.nearestFaceToPoint (nearPnt, null, target, p.getPosition ());
            p.setPosition (nearPnt);
         }       
         
         if(vertDistList.get (i).size ()>0) {
            Vertex3d vert0 = vertDistList.get (i).get (0).getVert ();
            double dist = vert0.distance (p.getPosition ());
            double par = (1/(var * Math.sqrt (2*Math.PI))) 
               * Math.exp (-0.5* (dist/var)*(dist/var));
            vert0.getPosition ().add (tr); 
   
            for(int j=1; j<vertDistList.get(i).size (); j++) {
              
               Vertex3d vert = vertDistList.get (i).get(j).getVert (); 
               double dist1 = vertDistList.get (i).get(j).getCost ();      
               double par1 = (1/(var * Math.sqrt (2*Math.PI))) 
               * Math.exp (-0.5* (dist1/var)*(dist1/var))/par;
               vert.getPosition ().scaledAdd (par1, tr);    
            }   
         }
      }
   }
   
   public static ArrayList<Vertex3d> checkSDF(
      ArrayList<Vertex3d> keyV, 
      PolygonalMesh mesh) {  
      
      ArrayList<Vertex3d> keyVTemp = new ArrayList<Vertex3d>(keyV);
      int sdfIter =3;
      if(mesh.numVertices ()<5000)
         sdfIter =2;
      else if(mesh.numVertices ()>50000)
         sdfIter =5;
      //System.out.println("sdfIter: "+ sdfIter); 
      for(int iter =0; iter<sdfIter;iter++) { //3 for 10k-20k-vertex meshes
         for(int i = 0; i < keyVTemp.size (); i++) {
            Vertex3d vert = keyVTemp.get (i);
            double sdf = ShapeDiameterFunction.computeShapeDiameter(vert, mesh, coneAngle, numRays);
            Vertex3d vertTemp  =  iterSDF(mesh, vert, sdf);
            if(Math.abs (vertTemp.pnt.z-keyV.get (i).pnt.z)<5)
               if(vert!= vertTemp && checkNormal(vert, vertTemp, mesh)==true)              
                  keyVTemp.set (i, vertTemp);             
         }     
      } 
      return keyVTemp;
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
                                                            
         double neighborSDF =  ShapeDiameterFunction.computeShapeDiameter(          
            vtxTail, mesh, coneAngle, numRays); 

         if(neighborSDF > sdfTemp +0.5) {                          
            sdfTemp = neighborSDF;
            vtxTemp = vtxTail;
         }                  
      }       
   
      return vtxTemp;        
   } 
    
   public static ArrayList<Vertex3d> snakeCurv(
      ArrayList<Vertex3d> keyV, 
      PolygonalMesh mesh) {      
           
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

         if(preVert.distance (vert)<2) {
           // System.out.println("Log: pre and vert are close"+ i);              
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
         
         if(geoList==null)
            continue;
         double geoDist;
         if(geoList.size ()==0) 
            geoDist =0;                          
         else
            geoDist = geoList.get (geoList.size ()-1).getCost ();

         // System.out.println("geoList "+ geoList.size());  

         double intE =0.05* geoDist* geoDist + 0.1* order2.norm()* order2.norm(); //2

         // System.out.println("i "+ i+ " " + "iter " +iter);  
         //System.out.println("intE "+ intE); 

         double extE = computeMinVtxCurvature(vert, mesh);

         // System.out.println("extE "+ extE); 
         double energy = intE +extE;
         Iterator<HalfEdge> heIterator = vert.getIncidentHalfEdges (); 

         while (heIterator.hasNext()) {
            HalfEdge he = heIterator.next();
            Vertex3d vtxTail = he.tail; 

            if(checkNormal(vert, vtxTail, mesh)==true) {
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

               double intEn = 0.05*geoDistTemp*geoDistTemp + 0.1*order2N.norm()*order2N.norm(); //2
               double extEn = computeMinVtxCurvature(vtxTail, mesh);
               double energyN = intEn +extEn;

               if(energyN+ 0.1 < energy) {                                               
                  energy = energyN;
                  vert = vtxTail; 
                //  geoList = geoListTemp;
               }  
            }
         }                    
         keyV.set (i, vert); 
         
//         mesh.setVertexColoringEnabled ();
//         for(int m=0; m<geoList.size (); m++)
//            mesh.setColor (geoList.get (m).getVert ().getIndex (), Color.red);
      }
      
      return keyV;
   }
    
   public static boolean checkNormal(Vertex3d vert, Vertex3d vtxTail, PolygonalMesh mesh) {
   // prevent verts from moving to a position facing more downward         
      Vector3d nrm0 = new Vector3d();
      vert.computeNormal (nrm0);
      Vector3d zDir = new Vector3d(0,0, 1);
      double angz0 = 180 * nrm0.angle (zDir)/Math.PI;
      if(angz0>60)
         return true;
      
      Vector3d nrm1 = new Vector3d();
      vtxTail.computeNormal (nrm1);
      Vector3d subN = new Vector3d();
      subN.sub (nrm1, nrm0);
      subN.normalize ();      
      double angz1 = 180 * nrm1.angle (zDir)/Math.PI;
     // double angm = 180 * nrm1.angle (nrm0)/Math.PI;
           
      if(subN.z>0.05 && angz1>60)// && angm>5) //
         return true;
      else 
         return false;
   }
   
   //https://computergraphics.stackexchange.com/questions/1718/
   //what-is-the-simplest-way-to-compute-principal-curvature-for-a-mesh-triangle     
    public static double computeMinVtxCurvature(Vertex3d vtx, PolygonalMesh mesh) {   
       Vector3d nrm1 = new Vector3d();
       vtx.computeNormal (nrm1);
       double minCurv=10;
       Iterator<HalfEdge> heIterator = vtx.getIncidentHalfEdges ();  
       while (heIterator.hasNext()) {
          HalfEdge he = heIterator.next();
          Vertex3d vtxTail = he.tail;
         
          Vector3d nrm2 = new Vector3d();
          vtxTail.computeNormal (nrm2);        
          Vector3d nrmSub = new Vector3d();
          nrmSub.sub (nrm2, nrm1);
          Vector3d pntSub = new Vector3d();
          pntSub.sub(vtxTail.pnt,vtx.pnt);
          double dist = vtx.distance (vtxTail);        
          double curv = nrmSub.dot(pntSub)/(dist*dist);
         
          if(curv<minCurv)
             minCurv=curv;
          }
       return minCurv;
    }
      
    public static void aveSmoothing(
       IntersectionContour contours, 
       PolygonalMesh mesh){
       
       ArrayList<Vertex3d> bdrVerts = new ArrayList<Vertex3d>();      
       BVFeatureQuery query = new BVFeatureQuery();  
      
       for(int i=0; i<contours.size (); i++) {
          Vertex3d nearestVert = query.nearestVertexToPoint (mesh, contours.get (i));               
          if(bdrVerts.contains (nearestVert)==false) {
             bdrVerts.add (nearestVert); 
             Point3d tempP = new Point3d();
             int neighborNo = 0;    
      
             Iterator<HalfEdge> heIterator = nearestVert.getIncidentHalfEdges ();       
             while (heIterator.hasNext()) {
                HalfEdge he = heIterator.next();
                Vertex3d vtxTail = he.tail;
                  
                tempP.add (vtxTail.pnt);          
                neighborNo++;
                }
             tempP.scale (1.0/neighborNo);       
             nearestVert.setPosition (tempP); 
             }         
          }
       
       int smoothIter = 2;
       if(mesh.numVertices ()<5000)
          smoothIter =0;
       else if(mesh.numVertices ()>50000)
          smoothIter =4;
       //System.out.println("smoothIter: "+ smoothIter); 
       for(int t=0; t<smoothIter; t++) { //2 for 10k-20k-vertex meshes
          for(int i=0; i<bdrVerts.size (); i++) {
             Vertex3d vert= bdrVerts.get (i);
             Iterator<HalfEdge> heIterator = vert.getIncidentHalfEdges ();   
             Point3d tempP0 = new Point3d();
             int neighborNo0 = 0;    
 
             while (heIterator.hasNext()) {
                HalfEdge he = heIterator.next();
                Vertex3d vtxTail = he.tail;               
                Point3d tempP2 = new Point3d();
                int neighborNo2 = 0;                 

                Iterator<HalfEdge> heIterator2 = vtxTail.getIncidentHalfEdges ();       
                while (heIterator2.hasNext()) {
                   HalfEdge he2 = heIterator2.next();
                   Vertex3d vtxTail2 = he2.tail;                   
                   tempP2.add (vtxTail2.pnt);
                   neighborNo2++;
                   
                   }
                tempP2.scale (1.0/neighborNo2);   
                vtxTail.setPosition (tempP2); 
                tempP0.add (vtxTail.pnt);          
                neighborNo0++;
             } 
             tempP0.scale (1.0/neighborNo0);           
             vert.setPosition (tempP0);           
             
          }     
       }       
    }
}


