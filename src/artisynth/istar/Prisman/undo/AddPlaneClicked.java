package artisynth.istar.Prisman.undo;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.HelperMathFunctions;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;

public class AddPlaneClicked extends AbstractAction {
   ReconstructionModel root = null;
   MechModel mechModel = null;
   Assist Assist = new Assist ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();  
   HelperMathFunctions mathHelper = new HelperMathFunctions();
   
   public AddPlaneClicked (ReconstructionModel root, MechModel model) {
      this.root = root;
      this.mechModel = model;
   }
   
   @Override
   public void actionPerformed (ActionEvent e) {
      PolygonalMesh plane = MeshFactory.createPlane (90, 90, 10, 10);
      FixedMeshBody planeMesh = new FixedMeshBody (plane);
      planeMesh.setName ("BonyContactPlane");
      mechModel.addMeshBody (planeMesh);
      
   }

}
