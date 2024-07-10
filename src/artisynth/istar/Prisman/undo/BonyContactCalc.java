package artisynth.istar.Prisman.undo;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;

import artisynth.core.mechmodels.MechModel;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.HelperMathFunctions;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.PolygonalMesh;

public class BonyContactCalc extends AbstractAction {
   ReconstructionModel root = null;
   MechModel mechModel = null;
   Assist Assist = new Assist ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();  
   HelperMathFunctions mathHelper = new HelperMathFunctions();
   
   public BonyContactCalc (ReconstructionModel root, MechModel model) {
      this.root = root;
      this.mechModel = model;
   }
   
   @Override
   public void actionPerformed (ActionEvent e) {
      PolygonalMesh plane = new PolygonalMesh ();
      plane = Assist.GetMesh (mechModel, "BonyContactPlane");
      
      PolygonalMesh mand = new PolygonalMesh ();
      mand = Assist.GetMesh (mechModel, "Mandible");
      
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector ();
      
      PolygonalMesh intersection = intersector.findIntersection (plane, mand);
      
      double area = intersection.computeArea ();
      
      System.out.println ("Surface area here is: " + area);
   }

}
