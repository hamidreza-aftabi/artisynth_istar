package artisynth.istar.MandibleRecon;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;
import javax.swing.JSlider;

import artisynth.core.mechmodels.MechModel;
import artisynth.istar.Prisman.ReconstructionModel;

/**
 * @author Matthew Mong Stiffness control for plate bending
 */
public class StiffnessControlClicked extends AbstractAction {
   /**
    * 
    */
   private static final long serialVersionUID = 7821057376088616928L;
   ReconstructionModel root;
   MechModel mechModel;

   public StiffnessControlClicked (ReconstructionModel root,
   MechModel mechModel) {
      putValue (NAME, "Plate Mandible");
      this.root = root;
      this.mechModel = mechModel;
   }

   @Override
   public void actionPerformed (ActionEvent e) {
      JSlider b = new JSlider (0, 20, 1);

      // paint the ticks and tracks
      b.setPaintTrack (true);
      b.setPaintTicks (true);
      b.setPaintLabels (true);

   }
}