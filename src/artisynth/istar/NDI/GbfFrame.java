package artisynth.istar.NDI;

import java.util.*;

/**
 * This class encapsulates frame data whose data was gathered at a single point in time
 */
public class GbfFrame extends GbfComponent {

   //! Frame data is sent as a set of GbfFrameDataItems
   ArrayList<GbfFrameDataItem> data;

   /**
    * Parses data from a BX2 reply that has been read into the buffer.
    */
   GbfFrame (BufferedReader reader, int dataItems) {
      data = new ArrayList<>();
      for (int i = 0; i < dataItems; i++) {
         GbfFrameDataItem framedata = new GbfFrameDataItem(reader);
         data.add(framedata);
      }
   }

   /**
    * Frees memory for any frameData that was allocated.
    */
   void dispose() {
   }

   /**
    * Repackages frame data on a per-tool basis.
    */
   ArrayList<ToolData> getToolData() {
      // The end goal is to flatten the data into the ToolData structures for client-side manipulation.
      ArrayList<ToolData> tools = new ArrayList<>();

      // System alerts are transmitted with each GbfFrameDataItem
      ArrayList<SystemAlert> gbfFrameDataItemAlerts = new ArrayList<>();

      // Active, Active-Wireless, and Passive tools collect data in different ways.
      // Thus, each frame of data must be divided into separate frames for each tool type.
      // BX2 transmits the data as one or more GbfFrameDataItems on each GbfFrame.
      for (int i = 0; i < data.size(); i++) {
         // Go through the GBF components on each GbfFrameDataItem
         for (int c = 0; c < data.get(i).frameData.components.size(); c++)	{
            GbfComponent component = data.get(i).frameData.components.get(c);
            short type = component.componentType;
            if (type == GbfComponent.Type.Data6D.ordinal()) {
               // Go through each 6D on this GBF component
               GbfData6D data6D = (GbfData6D)component;
               for (int j = 0; j < data6D.toolTransforms.size(); j++) {
                  // Append the 6D if the ToolData exists
                  boolean toolExists = false;
                  for (int t =0; t < tools.size(); t++) {
                     if (tools.get(t).transform.toolHandle ==
                         data6D.toolTransforms.get(j).toolHandle) {
                        toolExists = true;
                        tools.get(t).transform = data6D.toolTransforms.get(j);
                        break;
                     }
                  }
                  if (!toolExists) {
                     // Create a ToolData object with the frame and transform information
                     ToolData tool = new ToolData();
                     tool.dataIsNew = true;
                     tool.frameType = data.get(i).frameType;
                     tool.frameSequenceIndex = data.get(i).frameSequenceIndex;
                     tool.frameStatus = data.get(i).frameStatus;
                     tool.frameNumber = data.get(i).frameNumber;
                     tool.timespec_s = data.get(i).timespec_s;
                     tool.timespec_ns = data.get(i).timespec_ns;
                     tool.transform = data6D.toolTransforms.get(j);
                     tool.systemAlerts = gbfFrameDataItemAlerts;
                     tools.add (tool);
                  }
               }
            }
            else if (type == GbfComponent.Type.Data3D.ordinal()) {
               // Go through each 3D on this GBF component
               GbfData3D data3D = (GbfData3D)component;
               for (int j = 0; j < data3D.toolHandles.size(); j++) {
                  // Append the 3D information if the ToolData exists
                  boolean toolExists = false;
                  for (int t = 0; t < tools.size(); t++) {
                     if (tools.get(t).transform.toolHandle == data3D.toolHandles.get(j)) {
                        toolExists = true;
                        tools.get(t).markers = data3D.markers.get(j);
                        break;
                     }
                  }
                  if (!toolExists) {
                     // Create a ToolData object with the frame and marker information
                     ToolData tool = new ToolData();
                     tool.transform.toolHandle = (short)data3D.toolHandles.get(j); // don't forget the handle
                     tool.dataIsNew = true;
                     tool.frameType = data.get(i).frameType;
                     tool.frameSequenceIndex = data.get(i).frameSequenceIndex;
                     tool.frameStatus = data.get(i).frameStatus;
                     tool.frameNumber = data.get(i).frameNumber;
                     tool.timespec_s = data.get(i).timespec_s;
                     tool.timespec_ns = data.get(i).timespec_ns;
                     tool.markers = data3D.markers.get(j);
                     tool.systemAlerts = gbfFrameDataItemAlerts;
                     tools.add (tool);
                  }
               }
            }
            else if (type == GbfComponent.Type.Button1D.ordinal()) {
               GbfButton1D buttonData = (GbfButton1D)component;
               if (buttonData.data.size() > 0) {
                  // Append button data if the ToolData exists
                  boolean toolExists = false;
                  for (int t = 0; t < tools.size(); t++) {
                     if (tools.get(t).transform.toolHandle == buttonData.toolHandle) {
                        toolExists = true;
                        tools.get(t).buttons = buttonData.data;
                        break;
                     }
                  }
                  if (!toolExists) {
                     // Create a ToolData object with the button data and frame information
                     ToolData tool = new ToolData();
                     tool.transform.toolHandle = buttonData.toolHandle; // don't forget the handle
                     tool.dataIsNew = true;
                     tool.frameType = data.get(i).frameType;
                     tool.frameSequenceIndex = data.get(i).frameSequenceIndex;
                     tool.frameStatus = data.get(i).frameStatus;
                     tool.frameNumber = data.get(i).frameNumber;
                     tool.timespec_s = data.get(i).timespec_s;
                     tool.timespec_ns = data.get(i).timespec_ns;
                     tool.buttons = buttonData.data;
                     tool.systemAlerts = gbfFrameDataItemAlerts;
                     tools.add (tool);
                  }
               }
            }
            else if (type == GbfComponent.Type.SystemAlert.ordinal()) {
               // Store the alerts "globally" within this method so they are added to new ToolData
               GbfSystemAlert alert = (GbfSystemAlert)component;
               gbfFrameDataItemAlerts = alert.data;

               // Append the alert information to all ToolData from this GbfFrameDataItem
               for (int t = 0; t < tools.size(); t++) {
                  if (tools.get(t).frameNumber == data.get(i).frameNumber) {
                     tools.get(t).systemAlerts = gbfFrameDataItemAlerts;
                     break;
                  }
               }
            }
         } // process the next GbfComponent with more tool information
      } // process the next GbfFrameDataItem for a different tool type...
      return tools;
   }

   /**
    * Returns a string representation of the data for debugging purposes.
    */
   public String toString() {
      StringBuilder sb = new StringBuilder();
      sb.append ("-----GbfFrame \n");
      sb.append (super.toString());
      for (int i = 0; i < data.size(); i++) {
         sb.append (data.get(i).toString());
      }
      return sb.toString();
   }

};
