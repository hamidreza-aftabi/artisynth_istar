package artisynth.istar.NDI;

import java.util.*;
import java.io.*;
import java.net.*;

import maspack.util.*;

/**
 * Fakes an NDI device. Used for communication testing.
 */
public class NDIStubServer {
   
   int myPort;

   SystemCRC myCrc = new SystemCRC();

   String myString0Val = null;
   boolean myTrackingModeP = false;
   StreamThread myStreamThread;

   /**
    * Used for writing binary data
    */
   private class ByteWriter {
      DynamicByteArray buffer = new DynamicByteArray();

      public int numBytes() {
         return buffer.size();
      }

      public void clear() {
         buffer.clear();
      }

      void writeByte (int val) {
         buffer.add ((byte)val);
      }

      void writeShort (int val) {
         buffer.add ((byte)(val & 0xff));
         buffer.add ((byte)((val>>>8) & 0xff));
      }

      void writeShort (short val) {
         buffer.add ((byte)(val & 0xff));
         buffer.add ((byte)((val>>>8) & 0xff));
      }

      void writeInt (int val) {
         buffer.add ((byte)(val & 0xff));
         buffer.add ((byte)((val>>>8) & 0xff));
         buffer.add ((byte)((val>>>16) & 0xff));
         buffer.add ((byte)((val>>>24) & 0xff));
      }

      void writeFloat (float val) {
         writeInt (Float.floatToIntBits(val));
      }

      byte[] getBytes() {
         return buffer.getArray();
      }
   }

   private static int NEEDS_FREE = 0x1000;
   private static int OCCUPIED = 0x01;
   private static int INITIALIZED = 0x10;
   private static int ENABLED = 0x20;

   private class PortInfo {
      int status = 0;
      int handle = 0;

      PortInfo (int handle) {
         this.handle = handle;
      }
   }

   HashMap<Integer,PortInfo> myPortMap = new HashMap<>();

   NDIStubServer(int port) {
      myPort = port;
   }

   static final char CR = '\r';

   private void closeQuietly (Socket socket) {
      try {
         socket.close();
      }
      catch (IOException e) {
      }
   }

   public void sleepSeconds(double sec) {
      try {
         Thread.sleep ((int)(1000*sec));
      }
      catch (InterruptedException e) {
      }
   }

   Socket listenForConnection () {
      Socket socket = null;
      try (ServerSocket listenSocket = new ServerSocket(myPort)) {
         socket = listenSocket.accept();
      }
      catch (IOException e) {
         System.out.println ("Error: listenForConnection failed, "+e);
         if (socket != null) {
            closeQuietly (socket);
         }
      }
      return socket;
   }

   String readCommand (Socket socket) throws IOException {
      InputStream is = socket.getInputStream();
      StringBuilder sb = new StringBuilder();
      int c;
      while ((c = is.read()) != CR) {
         if (c == -1) {
            return null;
         }
         sb.append ((char)c);
      }
      return sb.toString();
   }

   boolean matchCommand (
      String str, String cmd, StringHolder options) throws IOException {
      if (str.startsWith (cmd+" ")) {
         int clen = cmd.length()+1;
         options.value = str.substring (clen, str.length());
         return true;
      }
      else {
         return false;
      }
   }

   void writeBytes (OutputStream os, String str) throws IOException {
      byte[] bytes = str.getBytes();
      os.write (bytes, 0, bytes.length);
   }

   void sendReply (OutputStream os, String reply) throws IOException {
      writeBytes (os, reply);
      writeBytes (
         os, Utils.shortToHexString ((short)myCrc.calculateCRC16 (reply)));
      os.write ((byte)CR);  
      os.flush();
   }

   void sendBX2Reply (OutputStream os, String options) throws IOException {
      // for now, ignore options, and just send a BX2 reply that corresponds to
      // the following sample CSV data:

      // #Tools, ToolInfo,         Frame#,    PortHandle, Face#, TransformStatus, 
      // 1,      UBC s/n:3CB34803, 969306835, Port:1,     0,     Enabled,         write
      //
      // Q0,      Qx,       Qy,       Qz,      Tx,      Ty,      Tz,      Error,    
      // 0.353514,-0.337378,-0.188594,0.851843,-189.144,-52.5834,-1699.78,0.536089,
      //
      // #Markers, Marker0.Status, Tx,      Ty,      Tz,       Marker1.Status, Tx,     Ty,      Tz,       
      // 4,        OK,             -214.093,-18.8788,-1719.77, OK,             -174.78,-26.1088,-1670.12, 
      //
      // Marker2.Status, Tx,      Ty,      Tz,       Marker3.Status, Tx,     Ty,       Tz
      // OK,             -199.985,-88.9995,-1727.89, OK,             -167.716,-76.3464,-1681.34
      ByteWriter hw = new ByteWriter(); // header writer
      ByteWriter dw = new ByteWriter(); // data writer
      
     // create data

      // write top-level container
      dw.writeShort (0x0001); // GBF version
      dw.writeShort (0x0001); // component count

      // 1 Frame component
      dw.writeShort (GbfComponent.Type.Frame.ordinal()); // component type
      dw.writeInt   (12+16+4+(12+4+32)+(12+4+4*16));   // component size
      dw.writeShort (0x0000); // item option (ignored)
      dw.writeInt   (1);      // item count (1 Frame to parse)

      // write frame
      dw.writeByte (0x02);    // frame type (Passive)
      dw.writeByte (0x00);    // sequence index
      dw.writeShort (0x00);   // status (OK)
      dw.writeInt (0x382E05BF);  // frame number
      dw.writeInt (0x577574CB);  // timestamp, seconds: Jun 30 2016
      dw.writeInt (0x2AD9A212);  // timestamp, nanoseconds

      // write frame component container
      dw.writeShort (0x0001); // GBF version
      dw.writeShort (0x0002); // Data6D and Data3d

      // 1 Data6D component

      dw.writeShort (GbfComponent.Type.Data6D.ordinal()); // component type 
      dw.writeInt   (12+4+32);   // component size
      dw.writeShort (0x0000); // item option (ignored)
      dw.writeInt   (1);      // item count (1 Frame)

      // Data6D

      dw.writeShort (0x0001); // tool handle 1
      dw.writeShort (0x2000); // status: OK, Face 1, transform present 

      dw.writeFloat (0.353514f); // q0
      dw.writeFloat (-0.337378f); // qx
      dw.writeFloat (-0.188594f); // qy
      dw.writeFloat (0.851843f); // qz
      dw.writeFloat (-189.144f); // tx
      dw.writeFloat (-52.5834f); // ty
      dw.writeFloat (-1699.78f); // tz
      dw.writeFloat (0.536089f); // error

      // 1 Data3D component

      dw.writeShort (GbfComponent.Type.Data3D.ordinal()); // component type
      dw.writeInt (12+4+4*16);   // component size
      dw.writeShort (0x0000); // item option (ignored)
      dw.writeInt   (1);      // item count

      // write Data3D

      dw.writeShort (0x0001); // tool handle 1
      dw.writeShort (0x0004); // num markers (4)

      dw.writeByte (0x00);    // status (OK)
      dw.writeByte (0x00);    // reserved
      dw.writeShort (0x00);   // index
      dw.writeFloat (-214.093f);   // pos.x
      dw.writeFloat (-18.8788f);   // pos.x
      dw.writeFloat (-1719.77f);   // pos.y

      dw.writeByte (0x00);    // status (OK)
      dw.writeByte (0x00);    // reserved
      dw.writeShort (0x01);   // index
      dw.writeFloat (-174.78f);   // pos.x
      dw.writeFloat (-26.1088f);   // pos.x
      dw.writeFloat (-1670.12f);   // pos.y
 
      dw.writeByte (0x00);    // status (OK)
      dw.writeByte (0x00);    // reserved
      dw.writeShort (0x02);   // index
      dw.writeFloat (-199.985f);   // pos.x
      dw.writeFloat (-88.9995f);   // pos.x
      dw.writeFloat (-1727.89f);   // pos.y

      dw.writeByte (0x00);    // status (OK)
      dw.writeByte (0x00);    // reserved
      dw.writeShort (0x03);   // index
      dw.writeFloat (-167.716f);   // pos.x
      dw.writeFloat (-76.3464f);   // pos.x
      dw.writeFloat (-1681.34f);   // pos.y

      // create header
      hw.writeShort (CombinedApi.START_SEQUENCE);
      hw.writeShort (dw.numBytes());
      hw.writeShort (myCrc.calculateCRC16 (hw.getBytes()));

      // calculate data CRC
      dw.writeShort (myCrc.calculateCRC16 (dw.getBytes()));

      os.write (hw.getBytes());
      os.write (dw.getBytes());
   }

   void sendTrackingData (OutputStream os, int flags) throws IOException {
      sendReply (
         os, "0101+07020+02043-03739+05706-018745-038074-170260+041270000003139C9FA54\n0000");
   }

   void handleCommand (String str, Socket socket) throws IOException {
      OutputStream os = socket.getOutputStream();
      StringHolder options = new StringHolder();
      if (matchCommand (str, "GET", options)) {
         if (options.value.equals ("Features.Firmware.Version")) {
            sendReply (os, "Features.Firmware.Version=88");
         }
         else if (options.value.equals ("Param.User.String0")) {
            sendReply (os, "Param.User.String0="+myString0Val);
         }
         else if (options.value.equals ("Info.Status.Gravity Vector")) {
            sendReply (os, "Info.Status.Gravity Vector=000000");
         }
         
      }
      else if (matchCommand (str, "APIREV", options)) {
         sendReply (os, "G.003.001");
      }
      else if (matchCommand (str, "INIT", options)) {
         sendReply (os, "OKAY");
      }
      else if (matchCommand (str, "TX", options)) {
         if (!myTrackingModeP) {
            sendReply (os, "ERROR0c");
         }
         else {
            int flags = Utils.hexStringToInt (options.value);
            sendTrackingData (os, flags);
         }
      }
      else if (matchCommand (str, "TSTART", options)) {
         myTrackingModeP = true;
         sendReply (os, "OKAY");
      }
      else if (matchCommand (str, "TSTOP", options)) {
         myTrackingModeP = false;
         sendReply (os, "OKAY");
      }
      else if (matchCommand (str, "SET", options)) {
         if (options.value.startsWith ("Param.User.String0")) {
            myString0Val =
               options.value.substring (options.value.indexOf('=')+1);
         }
         sendReply (os, "OKAY");
      }
      else if (matchCommand (str, "PHSR", options)) {
         int code = 0;
         if (!options.value.isEmpty()) {
            code = Integer.valueOf (options.value);
         }
         ArrayList<PortInfo> ports = new ArrayList<>();
         for (PortInfo pi : myPortMap.values()) {
            if (code == 0) {
               ports.add (pi);
            }
            else if (code == 1) {
               if ((pi.status & NEEDS_FREE) != 0) {
                  ports.add (pi);                  
               }
            }
            else if (code == 2) {
               if ((pi.status & OCCUPIED) != 0 &&
                   (pi.status & (INITIALIZED|ENABLED)) == 0) {
                  ports.add (pi);                  
               }
            }
            else if (code == 3) {
               if ((pi.status & OCCUPIED) != 0 &&
                   (pi.status & INITIALIZED) != 0 &&
                   (pi.status & ENABLED) == 0) {
                  ports.add (pi);                  
               }
            }
            else if (code == 4) {
               if ((pi.status & ENABLED) != 0) {
                  ports.add (pi);                  
               }
            }
         }
         StringBuilder sb = new StringBuilder();
         sb.append (Utils.byteToHexString ((byte)ports.size()));
         for (PortInfo pi : ports) {
            sb.append (Utils.byteToHexString ((byte)pi.handle));
            sb.append ('0');
            sb.append (Utils.byteToHexString ((byte)pi.status));
         }
         sendReply (os, sb.toString());
      }
      else if (matchCommand (str, "PINIT", options)) {
         int handle = Utils.hexStringToInt (options.value);
         PortInfo pi = myPortMap.get (handle);
         if (pi == null || (pi.status & OCCUPIED) == 0) {
            sendReply (os, "ERROR0F");
         }
         else {
            pi.status |= INITIALIZED;
            sendReply (os, "OKAY");
         }
      }
      else if (matchCommand (str, "PENA", options)) {
         int handle = Utils.hexStringToInt (options.value.substring(0,2));
         PortInfo pi = myPortMap.get (handle);
         if (pi == null || (pi.status & INITIALIZED) == 0) {
            sendReply (os, "ERROR0E");
         }
         else {
            pi.status |= ENABLED;
            sendReply (os, "OKAY");
         }
      }
      else if (matchCommand (str, "PVWR", options)) {
         int handle = Utils.hexStringToInt (options.value.substring(0,2));
         PortInfo pi = myPortMap.get (handle);
         if (pi == null) {
            sendReply (os, "ERROR08");
         }
         else {
            pi.status |= OCCUPIED;
            sendReply (os, "OKAY");
         }
      }
      else if (matchCommand (str, "PHINF", options)) {
         int handle = Utils.hexStringToInt (options.value.substring(0,2));
         PortInfo pi = myPortMap.get (handle);
         if (pi == null) {
            sendReply (os, "ERROR08");
         }
         else if ((pi.status & OCCUPIED) == 0) {
            sendReply (os, "UNOCCUPIED");
         }
         else {
            sendReply (os, "02000000UBC         R6 3CB3480300");
         }
      }
      else if (matchCommand (str, "PHRQ", options)) {
         int handle = myPortMap.size()+1;
         myPortMap.put (handle, new PortInfo(handle));
         sendReply (os, Utils.byteToHexString ((byte)handle));
      }
      else if (matchCommand (str, "STREAM", options)) {
         String[] opts = options.value.split(" ");
         String streamId = null;
         int port = -1;
         for (String opt : options.value.split(" ")) {
            if (!opt.isEmpty()) {
               if (opt.startsWith ("--id=")) {
                  streamId = opt.substring (5);
               }
               else if (opt.startsWith ("--port=")) {
                  port = Integer.valueOf (opt.substring(7));
               }
            }
         }
         myStreamThread = new StreamThread (port);
         myStreamThread.start();
         sendReply (os, "OKAY");         
      }
      else if (matchCommand (str, "BX2", options)) {
         sendBX2Reply (os, options.value);
      }
   }

   public class StreamThread extends Thread {

      int myPort;

      StreamThread (int port) {
         myPort = port;
      }

      public void run() {
         try {
            Socket socket = new Socket ("127.0.0.1", myPort);
            OutputStream os = socket.getOutputStream();
            while (true) {
               sendTrackingData (os, 0x0);
               sleepSeconds (0.1);
            }
         }
         catch (Exception e) {
            System.out.println ("error in streaming thread: "+e);
         }
      }
   }

   public void run() {
      System.out.println ("listening");
      while (true) {
         Socket socket = listenForConnection();
         System.out.println ("connected");
         try {
            String cmd;
            while ((cmd = readCommand(socket)) != null) {
               System.out.println ("> " + cmd);
               handleCommand (cmd, socket);
            }
         }
         catch (IOException e) {
            System.out.println ("IOException: "+e);
         }
         finally {
            closeQuietly (socket);
         }
      }
      
   }

   public static void main (String[] args) {
      NDIStubServer server = new NDIStubServer(8765);
      server.run();
   }

}
