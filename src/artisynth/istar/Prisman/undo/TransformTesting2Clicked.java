package artisynth.istar.Prisman.undo;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;

import artisynth.core.driver.Main;
import artisynth.core.gui.editorManager.UndoManager;
import artisynth.core.mechmodels.MechModel;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.ImprovedFormattedTextField;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.geometry.PolygonalMesh;

public class TransformTesting2Clicked extends AbstractAction {
   /**
    * 
    */
   private static final long serialVersionUID = -4357198968454315337L;
   /**
    * 
    */
   ReconstructionModel root = null;
   Assist Assist = new Assist ();
   MechModel mechModel = null;
   PolygonalMesh[] nonResectionFibulaMeshes;
   PolygonalMesh[] resectionFibulaMeshes;
   ImprovedFormattedTextField DonorDistanceProx;
   ImprovedFormattedTextField DonorDistanceDis;
   ImprovedFormattedTextField multiplier;
   double SAW_BLD_THICK;
   PolygonalMesh reconMand = null;

   public TransformTesting2Clicked (ReconstructionModel root, MechModel mechModel,
   ImprovedFormattedTextField DonorDistanceProx,
   ImprovedFormattedTextField DonorDistanceDis,
   ImprovedFormattedTextField multiplier, double SAW_BLD_THICK) {
      putValue (NAME, "Transform");
      this.root = root;
      this.mechModel = mechModel;
      this.DonorDistanceDis = DonorDistanceDis;
      this.DonorDistanceProx = DonorDistanceProx;
      this.multiplier = multiplier;
      this.SAW_BLD_THICK = SAW_BLD_THICK;
   }

   @Override
   public void actionPerformed (ActionEvent evt) {
      TestingTransform2 cmd = new TestingTransform2 (root, mechModel,
         DonorDistanceProx,
         DonorDistanceDis,
         multiplier, SAW_BLD_THICK);
      Main main = Main.getMain ();
      UndoManager undo = main.getUndoManager ();
      undo.execute (cmd);
   }
}
