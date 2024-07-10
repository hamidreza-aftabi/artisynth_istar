package artisynth.istar.NDI;

public class PortHandleInfo {

   String portHandle;
   String toolType;
   String toolId;
   String revision;
   String serialNumber;
   byte status;

   int ToolInPort = 0x01;
   int Switch1Closed = 0x02;
   int Switch2Closed = 0x04;
   int Switch3Closed = 0x08;
   int PortInitialized = 0x10;
   int PortEnabled = 0x20;
   int Reserved = 0x40;
   int CurrentSensed = 0x80;

   PortHandleInfo (String portHandle) {
      this (portHandle, (byte)0);
   }

   PortHandleInfo (String portHandle, byte status) {
      this.portHandle = portHandle;
      this.toolType = "";
      this.toolId = "";
      this.revision = "";
      this.serialNumber = "";
      this.status = status;
   }

   PortHandleInfo (
      String portHandle, String toolType, String toolId, String revision,
      String serialNumber, byte status) {

      this.portHandle = portHandle;
      this.toolType = toolType;
      this.toolId = toolId;
      this.revision = revision;
      this.serialNumber = serialNumber;
      this.status = status;
   }

   String getPortHandle() {
      return portHandle;
   }

   String getToolId() {
      return toolId;
   }

   String getRevision() {
      return revision;
   }

   String getSerialNumber() {
      return serialNumber;
   }

   String getStatus() {

      StringBuilder sbuf = new StringBuilder();
      if ((status & ToolInPort) != 0) {
         sbuf.append ("ToolInPort|");
      }
      if ((status & Switch1Closed) != 0) {
         sbuf.append ("Switch1Closed|");
      }
      if ((status & Switch2Closed) != 0) {
         sbuf.append ("Switch2Closed|");
      }
      if ((status & Switch3Closed) != 0) {
         sbuf.append ("Switch3Closed|");
      }
      if ((status & PortInitialized) != 0) {
         sbuf.append ("PortInitialized|");
      }
      if ((status & PortEnabled) != 0) {
         sbuf.append ("PortEnabled|");
      }
      if ((status & CurrentSensed) != 0) {
         sbuf.append ("CurrentSensed");
      }
      if (sbuf.charAt(sbuf.length()-1) == '|') {
         sbuf.deleteCharAt (sbuf.length()-1);
      }
      return sbuf.toString();
   }

   public String toString() {

      StringBuilder sbuf = new StringBuilder();
      sbuf.append ("portHandle[");
      sbuf.append (getPortHandle());
      sbuf.append ("].status=");
      sbuf.append (getStatus());
      return sbuf.toString();
   }
   
   public void dispose() {
   }
}
