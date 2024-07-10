package artisynth.istar.Prisman.undo;

import java.util.List;
import artisynth.core.mechmodels.*;
import artisynth.core.gui.editorManager.Command;



public class finalizeCommand implements Command {
	private String myName;
	private MechModel myModel;
	private List<FixedMeshBody> mySeg;
	private FixedMeshBody non;
	private FixedMeshBody recon;
	private int length;
	
	public finalizeCommand (String name, MechModel model, int numberOfSegments, List<FixedMeshBody> fibSeg, FixedMeshBody nonresect, FixedMeshBody reconstruction){
		myName = name;
		non = nonresect;
		myModel = model;
		mySeg = fibSeg;
		recon = reconstruction;
		length = numberOfSegments;
	}
	
	
	public void execute(){
			myModel.addMeshBody(recon);
			
			for (int i = 0; i < length; i++){
				mySeg.get(i).getRenderProps().setVisible(false);
			}
			non.getRenderProps().setVisible(false);
	}
	
	public void undo(){
		myModel.removeMeshBody(recon);
		for (int i = 0; i < length; i++){
				mySeg.get(i).getRenderProps().setVisible(true);
			}
			non.getRenderProps().setVisible(true);
	}
	
	public String getName(){
		return myName;
	}
}

