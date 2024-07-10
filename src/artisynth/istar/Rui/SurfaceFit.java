package artisynth.istar.Rui;

import java.awt.Color;
import java.io.*;
import java.util.*;
import java.util.Vector;

import artisynth.core.workspace.*;
import artisynth.core.mechmodels.*;
import artisynth.core.femmodels.*;
import artisynth.core.materials.*;
import artisynth.core.probes.*;
import artisynth.core.gui.*;

import maspack.util.*;
import maspack.matrix.*;
import maspack.geometry.*;
import maspack.render.*;
import maspack.render.Renderer.*;
import maspack.properties.*;

public class SurfaceFit extends RootModel {

   FixedMeshBody myReference;
   FixedMeshBody mySurface;
   MechModel myMech;

   private void addControlPoint (MechModel mech, double x, double y, double z) {
      Particle p = new Particle (0.0, new Point3d(x, y, z));
      p.setDynamic (false);
      mech.addParticle (p);
   }

   public void build (String[] args) {
      myMech = new MechModel ("mech");
      addModel (myMech);
      
      PolygonalMesh mesh = MeshFactory.createPlane (10.0, 2.5, 20, 5);
      myReference = new FixedMeshBody (mesh);
      mySurface = new FixedMeshBody (mesh.clone());
      myMech.addMeshBody (myReference);
      myMech.addMeshBody (mySurface);
      
      addControlPoint (myMech, -2.0, 0, 0.3);
      addControlPoint (myMech, 0, 0, 1);
      addControlPoint (myMech, 1.8, 0, -0.3);
      addControlPoint (myMech, 4.3, 0, -0.5);
      
      // control panel
      ControlPanel panel = new ControlPanel();
      panel.addWidget ("reference visible", myReference, "renderProps.visible");
      panel.addWidget ("surface visible", mySurface, "renderProps.visible");
      addControlPanel (panel);

      // render properties
      RenderProps.setSphericalPoints (myMech, 0.1, Color.RED);
      RenderProps.setDrawEdges (myReference, true);
      RenderProps.setLineColor (myReference, Color.GREEN);
      RenderProps.setFaceStyle (myReference, FaceStyle.NONE);
      RenderProps.setVisible (myReference, false);

      
      RenderProps.setDrawEdges (mySurface, true);
      RenderProps.setLineColor (mySurface, Color.CYAN);
      RenderProps.setFaceStyle (mySurface, FaceStyle.NONE);
   }
   
   public static void linearDeform(
      PolygonalMesh mesh,
      Point3d[] controlPoints){
      
      for(int i=0; i<controlPoints.length;i++) {
        
         Point3d vert = controlPoints[i];
         Point3d vertPre = new Point3d();
         if(i==0)
            vertPre = controlPoints[controlPoints.length-1];
         else
            vertPre =controlPoints[i-1];

         Point3d vertPost = new Point3d();
         if(i==controlPoints.length-1)
            vertPost = controlPoints[0];
         else
            vertPost =controlPoints[i+1];

         BVFeatureQuery query = new BVFeatureQuery(); 
         Point3d nearPnt = new Point3d();
         Face nearestFace = query.nearestFaceToPoint (nearPnt, null, mesh, vert);
         //double diff = vert.z- nearPnt.z;

         for(int k=0; k<3;k++) {
            Vertex3d vertF = nearestFace.getVertex (k);
            vertF.setVisited ();
            vertF.pnt.z = vert.z;     
            double distPreV = Math.sqrt (Math.pow (vert.x-vertPre.x, 2)
               + Math.pow (vert.y-vertPre.y, 2));
            double distPostV = Math.sqrt (Math.pow (vert.x-vertPost.x, 2)
               + Math.pow (vert.y-vertPost.y, 2));          

            Vector2d preV = new Vector2d();
            preV.x = vert.x -vertPre.x;
            preV.y = vert.y -vertPre.y;
            preV.normalize ();
            Vector2d Vpost = new Vector2d();
            Vpost.x = vertPost.x - vert.x;
            Vpost.y = vertPost.y - vert.y;
            Vpost.normalize ();
            
            Iterator<HalfEdge> heIterator = vertF.getIncidentHalfEdges ();       
            while (heIterator.hasNext()) {
               HalfEdge he = heIterator.next();
               Vertex3d vtxTail = he.tail;  
               
               if(vtxTail.isVisited () == false) {               
                  vtxTail.setVisited ();
                  Vector2d vec = new Vector2d();
                  vec.x = vtxTail.pnt.x-vert.x;
                  vec.y = vtxTail.pnt.y-vert.y;
                  double angPre =  Math.PI-vec.angle (preV);
                  double angPost = vec.angle (Vpost);
       
                  if(Math.abs(angPre -angPost)<0.01)
                     vtxTail.pnt.z = vert.z;
                  else if(angPre<angPost) {// && distPreV > distPre) {
                     Vector3d v1 = new Vector3d();
                     v1.sub (vtxTail.pnt, vertPre);                    
                     vtxTail.pnt.z = vertPre.z+
                     (vert.z-vertPre.z)*(v1.x *preV.x +v1.y*preV.y)/distPreV;
                  }
                  else if(angPre>angPost) {// && distPostV > distPost) {
                     Vector3d v1 = new Vector3d();
                     v1.sub (vtxTail.pnt, vert);                    
                     vtxTail.pnt.z = vert.z+
                     (vertPost.z-vert.z)*(v1.x *Vpost.x +v1.y*Vpost.y)/distPostV;
                  
                  }        
               }

                  Iterator<HalfEdge> heIterator2 = vtxTail.getIncidentHalfEdges ();       
                  while (heIterator2.hasNext()) {
                     HalfEdge he2 = heIterator2.next();
                     Vertex3d vtxTail2 = he2.tail;   
                    
                     if(vtxTail2.isVisited () == false) {                        
                        vtxTail2.setVisited ();                    
                        Vector2d vec2 = new Vector2d();
                        vec2.x = vtxTail2.pnt.x-vert.x;
                        vec2.y = vtxTail2.pnt.y-vert.y;
                        double angPre2 = Math.PI-vec2.angle (preV);
                        double angPost2 = vec2.angle (Vpost);                      
                        if(Math.abs(angPre2 -angPost2)<0.01)
                           vtxTail2.pnt.z =vert.z;
                        else if(angPre2<angPost2) {
                           Vector3d v1 = new Vector3d();
                           v1.sub (vtxTail2.pnt, vertPre);
                           vtxTail2.pnt.z =vertPre.z+
                           (vert.z-vertPre.z)*(v1.x *preV.x +v1.y*preV.y)/distPreV;                           
                        }
                        else if(angPre2>angPost2) {
                           Vector3d v1 = new Vector3d();
                           v1.sub (vtxTail2.pnt, vert);
                           vtxTail2.pnt.z = vert.z+
                           (vertPost.z-vert.z)*(v1.x *Vpost.x +v1.y*Vpost.y)/distPostV;  
       
                        }   
                        
                     }
                  }
               }
            

         }
      }
             
   }

   public static void gaussianDeform(
      PolygonalMesh mesh,
      Point3d[] controlPoints){
      
      double var = mesh.computeAverageEdgeLength ();
      for(int i=0; i<controlPoints.length;i++) {        
            Point3d vert = controlPoints[i];            
            BVFeatureQuery query = new BVFeatureQuery(); 
            Point3d nearPnt = new Point3d();
            Face nearestFace = query.nearestFaceToPoint (nearPnt, null, mesh, vert);            
            for(int k=0; k<3;k++) {
               Vertex3d vertF = nearestFace.getVertex (k);
               vertF.setVisited ();
               double diff = vert.z- nearPnt.z;
               double dist = Math.sqrt (Math.pow (vertF.pnt.x-nearPnt.x, 2)
                  +Math.pow (vertF.pnt.y-nearPnt.y, 2));
               double par = (1/(var * Math.sqrt (2*Math.PI))) 
               * Math.exp (-0.5* Math.pow (dist/var, 2));
               vertF.pnt.z = vert.z;
               
               Iterator<HalfEdge> heIterator = vertF.getIncidentHalfEdges ();       
               while (heIterator.hasNext()) {
                  HalfEdge he = heIterator.next();
                  Vertex3d vtxTail = he.tail;  
                  if(vtxTail.isVisited () == false) {
                     vtxTail.setVisited ();
                     double dist1 = Math.sqrt (Math.pow (vtxTail.pnt.x-nearPnt.x, 2)
                        +Math.pow (vtxTail.pnt.y-nearPnt.y, 2));
                     double par1 = (1/(var * Math.sqrt (2*Math.PI))) 
                     * Math.exp (-0.5* Math.pow (dist1/var, 2))/par;                
                        vtxTail.pnt.z += par1*diff;
                        }
                     Iterator<HalfEdge> heIterator2 = vtxTail.getIncidentHalfEdges ();       
                     while (heIterator2.hasNext()) {
                        HalfEdge he2 = heIterator2.next();
                        Vertex3d vtxTail2 = he2.tail;  
                        if(vtxTail2.isVisited () == false) {
                           vtxTail2.setVisited ();
                           double dist2 = Math.sqrt (Math.pow (vtxTail2.pnt.x-nearPnt.x, 2)
                              +Math.pow (vtxTail2.pnt.y-nearPnt.y, 2));
                           double par2 = (1/(var * Math.sqrt (2*Math.PI))) 
                           * Math.exp (-0.5* Math.pow (dist2/var, 2))/par;              
                              vtxTail2.pnt.z += par2*diff;
                           }
                        }
                     }
                                    
            }
         
         }     
   }
   
   void fitSurfaceLinearX (
      PolygonalMesh surface, Point3d[] controlPoints) {

      // sort x values of control points into ascending order
      double[] xvals = new double[controlPoints.length];
      int[] idxs = new int[controlPoints.length];
      for (int i=0; i<idxs.length; i++) {
         xvals[i] = controlPoints[i].x;
         idxs[i] = i;
      }
      ArraySort.quickSort (xvals, idxs);

      // find min and max x values for the mesh
      double xmin = Double.MAX_VALUE;
      double xmax = -Double.MAX_VALUE;
      for (Vertex3d vtx : surface.getVertices()) {
         if (vtx.getPosition().x < xmin) {
            xmin = vtx.getPosition().x;
         }
         else if (vtx.getPosition().x > xmax) {
            xmax = vtx.getPosition().x;
         }
      }

      // interpolate z values of each vertex based on control point intervals
      
      double x0 = xmin;
      double z0 = 0;
      for (int i=0; i<=xvals.length; i++) {
         double x1, z1;
         if (i < xvals.length) {
            x1 = xvals[i]; 
            z1 = controlPoints[idxs[i]].z;
         }
         else {
            x1 = xmax;
            z1 = 0;
         }

         for (Vertex3d vtx : surface.getVertices()) {
            Point3d vpos = vtx.getPosition();
            if (vpos.x >= x0 && vpos.x < x1) {
               // x is inside the interval - interpolate z
               vpos.z = z0 + (vpos.x-x0)/(x1-x0)*(z1-z0);
            }
         }
         x0 = x1;
         z0 = z1;
      }
   }

   void fitSurface (
      PolygonalMesh surface, Point3d[] controlPoints) {
      //fitSurfaceLinearX (surface, controlPoints);
      linearDeform (surface, controlPoints);
      //gaussianDeform (surface, controlPoints);
      
   }

   public void prerender (RenderList rlist) {
      // do surface fitting
      Point3d[] cpnts = new Point3d[myMech.particles().size()];
      int i = 0;
      for (Particle p : myMech.particles()) {
         cpnts[i++] = new Point3d(p.getPosition());
      }
      PolygonalMesh fitmesh = ((PolygonalMesh)myReference.getMesh()).clone();
      fitSurface (fitmesh, cpnts);
      PolygonalMesh surface = (PolygonalMesh)mySurface.getMesh();
      for (i=0; i<fitmesh.numVertices(); i++) {
         Vertex3d vs = surface.getVertex(i);
         vs.setPosition (fitmesh.getVertex(i).getPosition());
      }
      surface.notifyVertexPositionsModified();

      super.prerender(rlist);
   }

}
