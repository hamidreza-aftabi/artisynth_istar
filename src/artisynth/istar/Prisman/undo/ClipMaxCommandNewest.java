package artisynth.istar.Prisman.undo;

import artisynth.core.mechmodels.*;

import java.util.ArrayList;
import java.util.List;

import artisynth.core.gui.editorManager.Command;

//edit for n number planes
public class ClipMaxCommandNewest implements Command {
  private String myName;
  private FixedMeshBody mynonresect;
  private FixedMeshBody myresect;
  private List<FixedMeshBody> myplanes = new ArrayList<FixedMeshBody> ();
  private MechModel myModel;
  private RigidBody maxillaBody;
  private FixedMeshBody interbox;
  private List<FixedMeshBody> myboxes = new ArrayList<FixedMeshBody> ();

  public ClipMaxCommandNewest (String name, RigidBody maxilla,
  FixedMeshBody nonresection, FixedMeshBody resection, List<FixedMeshBody> planes,
  MechModel model, List<FixedMeshBody> boxes, FixedMeshBody intersectedbox) {
    myName = name;
    myModel = model;
    mynonresect = nonresection;
    myresect = resection;
    for (int i = 0; i < planes.size(); i++) {
        myplanes.add(i, planes.get(i));
    }
    maxillaBody = maxilla;

    for (int i = 0; i < boxes.size(); i++) {
        myboxes.add(i, boxes.get(i));
    }
    interbox = intersectedbox;
  }

  public void execute () {
    myModel.addMeshBody (mynonresect);
    myModel.addMeshBody (myresect);
    for (FixedMeshBody mybox : myboxes) {
        myModel.addMeshBody (mybox);
    }
    myModel.addMeshBody (interbox);
    maxillaBody.getRenderProps ().setVisible (false);
    myresect.getRenderProps ().setVisible (false);
    for (FixedMeshBody myplane : myplanes) {
      myplane.getRenderProps ().setVisible (false);
    }
    for (FixedMeshBody mybox : myboxes) {
      mybox.getRenderProps ().setVisible (false);
    }
    interbox.getRenderProps ().setVisible (false);

  }

  public void undo () {
    myModel.removeMeshBody (mynonresect);
    myModel.removeMeshBody (myresect);
    for (FixedMeshBody mybox : myboxes) {
      myModel.removeMeshBody (mybox);
    }
    myModel.removeMeshBody (interbox);
    maxillaBody.getRenderProps ().setVisible (true);
    for (FixedMeshBody myplane : myplanes) {
      myplane.getRenderProps ().setVisible (true);
    }

  }

  public String getName () {
    return myName;
  }
}
