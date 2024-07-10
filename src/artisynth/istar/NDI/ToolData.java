package artisynth.istar.NDI;

import java.util.*;
import maspack.util.*;

public class ToolData {

   static int PRECISION = 6;
   int frameNumber;
   Transform transform;
   int systemStatus;
   int portStatus;
   byte frameType;
   byte frameSequenceIndex;
   int frameStatus;
   int timespec_s;
   int timespec_ns;

   ArrayList<MarkerData> markers;
   DynamicByteArray buttons;
   ArrayList<SystemAlert> systemAlerts;   

   boolean dataIsNew;
   String toolInfo;

   public ToolData() {
	frameStatus = 0;
	frameNumber = 0;
	systemStatus = 0;
	portStatus = 0;
	dataIsNew = false;
	frameType = 0;
	frameSequenceIndex = 0;
	timespec_s = 0;
	timespec_ns = 0;
	toolInfo = "";
        transform = new Transform();
        markers = new ArrayList<>();
        buttons = new DynamicByteArray();
        systemAlerts = new ArrayList<>();
   }

   void set (ToolData tool) {
      frameNumber = tool.frameNumber;
      frameStatus = tool.frameStatus;
      systemStatus = tool.systemStatus;
      portStatus = tool.portStatus;
      dataIsNew = tool.dataIsNew;
      frameType = tool.frameType;
      frameSequenceIndex = tool.frameSequenceIndex;
      timespec_s = tool.timespec_s;
      timespec_ns = tool.timespec_ns;
      toolInfo = tool.toolInfo;
      transform = new Transform(tool.transform);
      markers = new ArrayList<>(tool.markers);
      systemAlerts = new ArrayList<>(tool.systemAlerts);
      buttons = new DynamicByteArray (tool.buttons);
   }

   // System status flags

   public static class SystemStatus {
      int CommSyncError = 0x0000;
      int ProcessingException = 0x0004;
      // Reserved 0x0008, 0x0010
      int PortOccupied = 0x0020;
      int PortUnoccupied = 0x0040;
      int DiagnosticPending = 0x0080;
      int TemperatureOutOfRange = 0x0100;

      String toString (int status) {
         StringBuilder sb = new StringBuilder();
         if ((status & CommSyncError) != 0) {
            sb.append ("CommSyncError|");
         }
         if ((status & ProcessingException) != 0) {
            sb.append("ProcessingException|");
         }
         if ((status & PortOccupied) != 0) {
            sb.append("PortOccupied|");
         }
         if ((status & PortUnoccupied) != 0) {
            sb.append("PortUnoccupied|");
         }
         if ((status & DiagnosticPending) != 0) {
            sb.append("DiagnosticPending|");
         }
         if ((status & TemperatureOutOfRange) != 0) {
            sb.append("TemperatureOutOfRange");
         }
         if (sb.charAt (sb.length()-1) == '|') {
            sb.deleteCharAt (sb.length()-1);
         }
         return sb.toString();
      }
   };

   public static enum FrameType {
      Dummy,
      ActiveWireless,
      Passive,
      Active,
      Laser,
      Illuminated,
      Background,
      Magnetic;

      public static FrameType findValue (int num) {
         if (num < values().length) {
            return values()[num];
         }
         else {
            return null;
         }
      }
   };

   public static enum ButtonState {
      Open,
      Closed;

      public static ButtonState findValue (int num) {
         if (num < values().length) {
            return values()[num];
         }
         else {
            return null;
         }
      }
   };
}
