package artisynth.istar.Rui;

import java.awt.Color;
import artisynth.core.workspace.*;
import artisynth.core.mechmodels.*;
import artisynth.core.gui.*;

import maspack.util.*;
import maspack.matrix.*;
import maspack.geometry.*;
import maspack.render.*;
import maspack.render.Renderer.*;

public class TPSSurfaceFit extends RootModel {

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
      
      PolygonalMesh mesh = MeshFactory.createPlane (20.0, 2.5, 20, 5);
      myReference = new FixedMeshBody (mesh);
      mySurface = new FixedMeshBody (mesh.clone());
      myMech.addMeshBody (myReference);
      myMech.addMeshBody (mySurface);
           
      addControlPoint (myMech, -9.0, 1, 4);
      addControlPoint (myMech, -6, -1, 0);
      addControlPoint (myMech, -5.5, 1, 0.1);
      addControlPoint (myMech, -2, -1, 0.6);
      addControlPoint (myMech, -1.5, 1, 0);
      addControlPoint (myMech, -1, -1, -0.2);
      addControlPoint (myMech, -0.5, -1, 0.2);
      addControlPoint (myMech, 0, -1, -0.2);
      addControlPoint (myMech, 0.5, -1, 0.2);
      addControlPoint (myMech, 1, -1, -0.2);
      addControlPoint (myMech, 1.5, 1, 0);
      addControlPoint (myMech, 2, -1, 0.1);
      addControlPoint (myMech, 5.5, 1, 0.1);
      addControlPoint (myMech, 6, -1, 0);
      addControlPoint (myMech, 8, -1, 0);
      addControlPoint (myMech, 8, 1, 0);
      addControlPoint (myMech, 9.0, 1, 4);
      //addControlPoint (myMech, 10, 1, 1);
      
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
   
   public static void thinPlateSpline(
      PolygonalMesh mesh,
      Point3d[] controlPoints){
      
      int ctrlPntSize = controlPoints.length;
      MatrixNd b = new MatrixNd(ctrlPntSize+3,1);
      for(int i=0; i<ctrlPntSize; i++) {          
         b.set (i, 0, controlPoints[i].z);
         }
           
      MatrixNd X = new MatrixNd(ctrlPntSize+3,1);      
      MatrixNd A = new MatrixNd(ctrlPntSize+3,ctrlPntSize+3);
      
      for(int i=0; i<ctrlPntSize; i++) {  
         for(int j=i+1; j<ctrlPntSize; j++) {   
            double dist = Math.sqrt (Math.pow (controlPoints[i].x - controlPoints[j].x,2) +
               Math.pow (controlPoints[i].y - controlPoints[j].y,2));
            double u = Math.pow (dist, 2) * Math.log (dist);            
            // System.out.println(i+ " dist "+ dist);                      
            A.set (i, j, u);  
            A.set (j, i, u); 
            }         
         A.set (i, ctrlPntSize, 1);
         A.set (i, ctrlPntSize+1, controlPoints[i].x);
         A.set (i, ctrlPntSize+2, controlPoints[i].y);
         
         A.set (ctrlPntSize, i, 1);
         A.set (ctrlPntSize+1, i, controlPoints[i].x);
         A.set (ctrlPntSize+2, i, controlPoints[i].y);
         }
      
      LUDecomposition LUD = new LUDecomposition(A);
      LUD.solve(X,b);
      
    A.writeToFile ("/Users/GladysYang/eclipse_workspace/artisynth_projects/src/artisynth/models/Rui/A.txt", "%g");
    X.writeToFile ("/Users/GladysYang/eclipse_workspace/artisynth_projects/src/artisynth/models/Rui/X.txt", "%g");
    b.writeToFile ("/Users/GladysYang/eclipse_workspace/artisynth_projects/src/artisynth/models/Rui/b.txt", "%g");
    

      for(int i=0; i<mesh.numVertices (); i++) {
         Vertex3d v = mesh.getVertex (i);
         
         v.pnt.z= X.get (ctrlPntSize, 0) + v.pnt.x * X.get (ctrlPntSize+1, 0) +
                  v.pnt.y * X.get (ctrlPntSize+2, 0);
         for(int j=0; j<ctrlPntSize; j++) {
            double dist = Math.sqrt (Math.pow (v.pnt.x - controlPoints[j].x,2) +
               Math.pow (v.pnt.y - controlPoints[j].y,2));
            double u = Math.pow (dist, 2) * Math.log (dist);
            v.pnt.z+= X.get (j, 0) * u;
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
      
      thinPlateSpline(surface, controlPoints);
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
