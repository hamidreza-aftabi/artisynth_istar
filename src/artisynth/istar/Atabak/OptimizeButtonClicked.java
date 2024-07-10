package artisynth.istar.Atabak;

import java.awt.event.ActionEvent;
import java.util.ArrayList;

import javax.swing.AbstractAction;

import artisynth.core.driver.Main;
import artisynth.core.gui.editorManager.UndoManager;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;




/**
 * @author Atabak Eghbal Abstract action for placing donor segments based on optimized plane
 * meshes present in a mech model
 *
 */

public class OptimizeButtonClicked extends AbstractAction{

   /**
    * This ID is nothing important. Don't worry about it
    */
   private static final long serialVersionUID = 916589320159290385L;


   MechModel mechModel = null;
   ReconstructionModel root = null;
   Assist Assist = new Assist ();
   FixedMeshBody resectionMeshBody, nonResectionMeshBody;
   ArrayList<FixedMeshBody> cuttingplanes = new ArrayList<> ();   
   ImprovedFormattedTextField numberOfSegments = null;
   ImprovedFormattedTextField DonorDistanceProx = null;
   ImprovedFormattedTextField DonorDistanceDis = null;
   private double SAW_BLD_THICK;

   public OptimizeButtonClicked(ReconstructionModel root,
   MechModel mechModel,
   ImprovedFormattedTextField DonorDistanceProx,
   ImprovedFormattedTextField DonorDistanceDis,ImprovedFormattedTextField numberOfSegments, double SAW_BLD_THICK) {
      
      putValue (NAME, "Just Click It");
      this.mechModel = mechModel;
      this.root = root;
      this.numberOfSegments = numberOfSegments;
      this.DonorDistanceProx = DonorDistanceProx;
      this.DonorDistanceDis = DonorDistanceDis;
      this.SAW_BLD_THICK = SAW_BLD_THICK;
   }
   
   @Override
   public void actionPerformed (ActionEvent e) {

    
      Optimize cmd = new Optimize (root, mechModel, DonorDistanceProx, DonorDistanceDis, numberOfSegments, SAW_BLD_THICK);
      Main main = Main.getMain ();
      UndoManager undo = main.getUndoManager ();
      undo.execute (cmd);      
      
   }

}
