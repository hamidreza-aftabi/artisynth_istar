//----------------------------------------------------------------------------
//
//  Copyright (C) 2017, Northern Digital Inc. All rights reserved.
//
//  All Northern Digital Inc. ("NDI") Media and/or Sample Code and/or Sample Code
//  Documentation (collectively referred to as "Sample Code") is licensed and provided "as
//  is" without warranty of any kind. The licensee, by use of the Sample Code, warrants to
//  NDI that the Sample Code is fit for the use and purpose for which the licensee intends to
//  use the Sample Code. NDI makes no warranties, express or implied, that the functions
//  contained in the Sample Code will meet the licensee's requirements or that the operation
//  of the programs contained therein will be error free. This warranty as expressed herein is
//  exclusive and NDI expressly disclaims any and all express and/or implied, in fact or in
//  law, warranties, representations, and conditions of every kind pertaining in any way to
//  the Sample Code licensed and provided by NDI hereunder, including without limitation,
//  each warranty and/or condition of quality, merchantability, description, operation,
//  adequacy, suitability, fitness for particular purpose, title, interference with use or
//  enjoyment, and/or non infringement, whether express or implied by statute, common law,
//  usage of trade, course of dealing, custom, or otherwise. No NDI dealer, distributor, agent
//  or employee is authorized to make any modification or addition to this warranty.
//  In no event shall NDI nor any of its employees be liable for any direct, indirect,
//  incidental, special, exemplary, or consequential damages, sundry damages or any
//  damages whatsoever, including, but not limited to, procurement of substitute goods or
//  services, loss of use, data or profits, or business interruption, however caused. In no
//  event shall NDI's liability to the licensee exceed the amount paid by the licensee for the
//  Sample Code or any NDI products that accompany the Sample Code. The said limitations
//  and exclusions of liability shall apply whether or not any such damages are construed as
//  arising from a breach of a representation, warranty, guarantee, covenant, obligation,
//  condition or fundamental term or on any theory of liability, whether in contract, strict
//  liability, or tort (including negligence or otherwise) arising in any way out of the use of
//  the Sample Code even if advised of the possibility of such damage. In no event shall
//  NDI be liable for any claims, losses, damages, judgments, costs, awards, expenses or
//  liabilities of any kind whatsoever arising directly or indirectly from any injury to person
//  or property, arising from the Sample Code or any use thereof
//
//----------------------------------------------------------------------------

package artisynth.istar.NDI;

// #include <fstream>
// #include <iomanip>
// #include <iostream>
// #include <sstream>
// #include <thread>

// #include "BufferedReader.h"
// #include "CombinedApi.h"
// #include "ComConnection.h"
// #include "GbfContainer.h"
// #include "GbfFrame.h"
// #include "SystemCRC.h"
// #include "TcpConnection.h"
// #include "UdpSocket.h"

import java.util.*;
import java.io.*;

import artisynth.istar.NDI.Connection.StreamingProtocol;

/**
 * This class encapsulates communication with NDI devices using the Combined API (CAPI).
 *
 * <p>
 * This class encapsulates binary and string parsing required to send/receive
 * commands.  This class does not provide an exhaustive implementation of every
 * API call, it does not implement every option available for some
 * commmands. It is intended as a supplement to the API guide, with working
 * sample code that compiles and runs cross platform
 * (Windows/Mac/Linux). Performance concerns were secondary to readability and
 * portability. Complexities like threading were intentionally avoided.  We
 * hope you will find this sample useful. Happy coding!
 */
public class CombinedApi {
   
   Connection connection = null;
   HashMap<String,Connection> streamMap;
   SystemCRC crcValidator;

   //! Same as Connection.StreamingProtocol, except exposed in a public header
   public static enum Protocol {
      TCP,
      UDP
   };


   //! Used by portHandleSearch() to filter the returned data.
   public static enum PortHandleSearchRequestOption {
      All,
      PortsToFree,
      NotInit,
      NotEnabled,
      Enabled;

      public static PortHandleSearchRequestOption findValue (int num) {
         if (num < values().length) {
            return values()[num];
         }
         else {
            return null;
         }
      }
   };

   public static enum ToolTrackingPriority {
      Static ('S'),
      Dynamic ('D'),
      ButtonBox ('B');

      char charVal;

      ToolTrackingPriority (char c) {
         charVal = c;
      }

      public char getCharVal() {
         return charVal;
      }
   };   

   public static enum TrackingReplyOption {
      TransformData (0x0001),
      ToolAndMarkerData (0x0002),
      SingleStray3D (0x0004),
      Tool3Ds (0x0008),
      AllTransforms (0x0800),
      PassiveStrays (0x1000);

      int mask;

      TrackingReplyOption (int mask) {
         this.mask = mask;
      }
      
      public int getMask() {
         return mask;
      }
   };      

   static char CR = '\r';

   //! Indicates the start of a BX or BX2 reply
   static short START_SEQUENCE = (short)0xA5C4;

   //! Indicates the start of an extended reply (eg. video capture)
   static short START_SEQUENCE_VCAP = (short)0xA5C8;

   //! Indicates the start of a streaming reply
   static short START_SEQUENCE_STREAMING = (short)0xB5D4;

   //! To avoid confusing error code 01 with warning 01, use this offset: 1001 is a warning, and 0001 is an error.
   static final int WARNING_CODE_OFFSET = 1000;

   /**
    * Creates a new instance of the CAPIcommand object.
    */
   public CombinedApi() {
      connection = null;
      streamMap = new HashMap<>();
      crcValidator = new SystemCRC();
   }

   /**
    * Disconnect and cleanup.
    */
   public void dispose() {
      stopStreaming(); // USTREAM and free any open streams before closing the _connection
      connection.dispose();
   }

   public String getVersion() {
      return "1.5.1";
   }

   /**
    * Connects to an ethernet device with the given hostname:port.
    * 
    * @param hostname The hostname, IP address, or COM# of the
    * device. Eg. "P9-B0103.local", "169.254.8.50", or "COM10"
    * @returns This method returns zero for success, or an error code.
    */
   public int connect (String hostname) {
      System.out.println ("Connecting to " + hostname + " ...");

      if (connection != null) {
         connection.dispose();
         connection = null;
      }

      int errorCode = -1;

      // Create a new TcpConnection
      connection = new TcpConnection(hostname);
      errorCode = connection.isConnected() ? 0 : -1;

      return errorCode;
   }

   /**
    * Prints firmware version information using APIREV.
    * 
    * @returns The result of the APIREV command.
    */
   public String getApiRevision() {
      // Send the APIREV command
      sendCommand("APIREV ");
      // Return the raw string so the client can parse the version
      return readResponse();
   }

   /**
    * Initializes the system with INIT.
    * 
    * @returns Zero for success, or the error code associated with the command.
    */
   public int initialize() {
      // Send the INIT command
      sendCommand("INIT ");
      return getErrorCodeFromResponse(readResponse());
   }

   /**
    * Search for port handles using PHSR.
    * 
    * @param option The PortHandleSearchRequestOption that defines what data is returned.
    * @returns A list of PortHandleInfo objects, or an empty vector if an error occurred.
    */
   public ArrayList<PortHandleInfo> portHandleSearchRequest (PortHandleSearchRequestOption option) {

      // Send the PHSR command
      sendCommand("PHSR "+Utils.intToString(option.ordinal(),2));

      // If an error occurred, return an empty vector
      String response = readResponse();
      int errorCode = getErrorCodeFromResponse(response);
      ArrayList<PortHandleInfo> portHandleInfoVector = new ArrayList<>();
      if (errorCode != 0) {
         System.out.println (response+" - "+errorToString(errorCode));
         return portHandleInfoVector;
      }

      // Return the response as an ArrayList of easy to use objects
      int numPortHandles = Utils.hexStringToInt(response.substring(0,2));
      for (int i = 0; i < numPortHandles; i ++) {
         int off = i*5;
         PortHandleInfo phi = new PortHandleInfo (
            response.substring(off+2, off+4),
            (byte)Utils.hexStringToInt(response.substring(off+4, off+7)));
         portHandleInfoVector.add (phi);
      }
      return portHandleInfoVector;
   }

   /**
    * Frees the specified port handle using PHF.
    * 
    * @param portHandle The port handle to free.
    * @returns Zero for success, -1 if portHandle is invalid, or the error code associated with the command.
    */
   public int portHandleFree(String portHandle) {
      // If the port handle is invalid, print an error message and return
      if (portHandle.length() != 2) {
         return -1;
      }

      // Send the PHF command
      sendCommand("PHF "+portHandle);
      return getErrorCodeFromResponse(readResponse());
   }

   /**
    * Port handle request with default values.
    */
   public int portHandleRequest () {
      return portHandleRequest ("********", "*", "1", "00", "**");
   }

   /**
    * Requests a port handle using PHRQ.
    * 
    * @param hardwareDevice Eight character name. Use wildcards "********" or a device name returned by PHINF
    * @param systemType One character defining the system type. Use a wildcard "*"
    * @param toolType One character defining the tool type. Wired="0" or Wireless="1" (passive or active-wireless)
    * @param portNumber Two characters defining the port number: [01-03] for wired tools, 00 or ** for wireless tools.
    * @param dummyTool If you will use PVWR to load a tool definition file, use wildcards: "**"
    *				    If ToolType=Wired, either "01" or "02" adds the active wired dummy tool.
    *                  If ToolType=Wireless, "01" adds the passive dummy tool, "02" adds the active wireless dummy tool.
    * @returns The requested port handle as a positive integer, or a negative integer error code.
    */
   public int portHandleRequest(String hardwareDevice, String systemType, String toolType, String portNumber, String dummyTool) {
      // Send the PHRQ command
      sendCommand(
         "PHRQ "+hardwareDevice+systemType+toolType+portNumber+dummyTool);

      // Return the requested port handle or an error code
      String response = readResponse();
      int errorCode = getErrorCodeFromResponse(response);
      if (errorCode == 0) {
         return Utils.hexStringToInt(response);
      }
      else {
         return errorCode;
      }
   }

   /**
    * Initializes a port handle using PINIT.
    * @param portHandle The port handle to initialize.
    * @returns Zero for success, -1 if portHandle is invalid, or the error code associated with the command.
    */
   public int portHandleInitialize(String portHandle) {
      
      // If the port handle is invalid, print an error message and return
      if (portHandle.length() != 2) {
         return -1;
      }

      // Send the PINIT command
      sendCommand ("PINIT "+portHandle);
      return getErrorCodeFromResponse(readResponse());
   }

   /**
    * Enables a port handle using PENA.
    * @param portHandle The portHandle to enable.
    * @param priority An enum value describing the type of tool.
    * @returns Zero for success, -1 if portHandle is invalid, or the error code associated with the command.
    */
   public int portHandleEnable(String portHandle, ToolTrackingPriority priority) {
      // If the port handle is invalid, print an error message and return
      if (portHandle.length() != 2) {
         return -1;
      }

      // Send the PENA command
      sendCommand ("PENA "+portHandle+priority.getCharVal());
      return getErrorCodeFromResponse(readResponse());
   }

   /**
    * Enables a port handle using PENA with dynamic priority.
    * @param portHandle The portHandle to enable.
    * @returns Zero for success, -1 if portHandle is invalid, or the error code associated with the command.
    */
   public int portHandleEnable(String portHandle) {
      return portHandleEnable (portHandle, ToolTrackingPriority.Dynamic);
   }

   /**
    * Retrieves tool information using PHINF.
    * @param portHandle The port handle to get information about
    * @returns A PortHandleInfo object with the information about the tool loaded in the given port.
    *          This method returns and empty PortHandleInfo objevct if the given portHandle is invalid,
    *          there is no tool loaded at the port, or an error occurs.
    */

   public PortHandleInfo portHandleInfo(String portHandle) {

      // If the port handle is invalid, print an error message and return
      if (portHandle.length() != 2) {
         System.out.println ("Invalid port handle: " + portHandle);
         return new PortHandleInfo(portHandle);
      }

      // Send the PHINF command
      sendCommand("PHINF "+portHandle);

      // If there is no tool loaded, return an empty PortHandleInfo
      String response = readResponse();
      int errorCode = getErrorCodeFromResponse(response);
      if (errorCode != 0) {
         System.out.println (
            response  + " - " + errorToString(errorCode));
         return new PortHandleInfo(portHandle);
      }
      else if (substr(response,0,10).equals("UNOCCUPIED")) {
         System.out.println (
            "No tool loaded at port: " + portHandle);
         return new PortHandleInfo(portHandle);
      }

      // Parse the information from the response
      String toolType = substr(response,0,8);
      String toolId = substr(response,8,12).trim();
      String revision = substr(response,20,3);
      String serialNumber = substr(response,23,8);
      byte status = (byte)Utils.hexStringToInt(substr(response,31,2));
      return new PortHandleInfo(portHandle, toolType, toolId, revision, serialNumber, status);
   }

   /**
    * Loads a dummy passive tool used to track stray 3Ds.
    * @details TSTART will fail if a dummy tool and regular tools of the same type are loaded and enabled.
    * @returns The requested port handle as a positive integer, or a negative integer error code.
    */
   public int loadPassiveDummyTool() {
      return portHandleRequest("********", "*", "1", "00", "01");
   }

   /**
    * Loads a dummy active wireless tool used to track stray 3Ds.
    * @details TSTART will fail if a dummy tool and regular tools of the same type are loaded and enabled.
    * @returns The requested port handle as a positive integer, or a negative integer error code.
    */
   public int loadActiveWirelessDummyTool() {
      return portHandleRequest("********", "*", "1", "00", "02");
   }

   /**
    * Loads a dummy active tool used to track stray 3Ds.
    * @details TSTART will fail if a dummy tool and regular tools of the same type are loaded and enabled.
    * @returns The requested port handle as a positive integer, or a negative integer error code.
    */
   public int loadActiveDummyTool() {
      return portHandleRequest("********", "*", "0", "00", "01");
   }

   /**
    * Loads a tool definition file (.rom) to a port using PVWR.
    * @param romFilePath The path to the .rom file to load.
    * @param portHandle The port handle (2 hex chars) that was previously requested.
    */
   public void loadSromToPort(String romFilePath, int portHandle) {

      // If the port handle is invalid, print an error message and return
      if (portHandle < 0) {
         System.out.println ("Invalid port handle: "+portHandle);
         return;
      }

      // If the .rom file cannot be opened, print an error message and return
      FileInputStream fis = null;
      try {
         fis = new FileInputStream (romFilePath);
      }
      catch (IOException e)  {
         System.out.println ("Cannot open file: " + romFilePath +", "+e);
      }

      // Read the entire file and convert it to ASCII hex characters
      StringBuilder romstr = new StringBuilder();
      int b = 0;
      try {
         while ((b = fis.read()) != -1) {
            romstr.append (Utils.byteToHexString ((byte)b));
         }
      }
      catch (IOException e) {
         System.out.println ("Error reading file: " + romFilePath +", "+e);
         return;
      }
      finally {
         try {
            fis.close();
         }
         catch (Exception e) {
         }
      }
      // Tool data is sent in chunks of 128 hex characters (64-bytes).
      // It must be an integer number of chunks, padded with zeroes at the end.
      int messageSizeChars = 128;
      int messageSizeBytes = 64;
      int remainder = romstr.length() % messageSizeChars;
      for (int i=0; i<messageSizeChars - remainder; i++) {
         romstr.append ('0');
      }
      int totalIterations = (int)romstr.length()/messageSizeChars;
      String command = "";
      int errorCode = 0;
      String tooldef = romstr.toString();
      for (int i = 0; i < totalIterations; i++) {
         // Send the PVWR command
         command =  "PVWR "+Utils.intToHexString(portHandle, 2);
         // Pass the startAddress as a fixed width four characters of hex padded with zeroes
         command += Utils.intToHexString(i * messageSizeBytes, 4);
         command += substr(tooldef,i*messageSizeChars, messageSizeChars);
         sendCommand(command);

         // If we run into an error, print something before exiting
         errorCode = getErrorCodeFromResponse(readResponse());
         if (errorCode != 0) {
            System.out.println ("PVWR returned error: " + errorToString(errorCode));
            return;
         }
      }
   }

   /**
    * Enters tracking mode using TSTART.
    * @returns Zero for success, or the error code associated with the command.
    */
   public int startTracking() {
      // Send the TSTART command
      sendCommand("TSTART ");
      return getErrorCodeFromResponse(readResponse());
   }

   /**
    * Exits tracking mode using TSTOP.
    * @returns Zero for success, or the error code associated with the command.
    */
   public int stopTracking() {
      // Send the TSTOP command
      sendCommand ("TSTOP ");
      return getErrorCodeFromResponse(readResponse());
   }

   private class StreamListenerThread extends Thread {

      TcpConnection myListener;
      int myPort;

      StreamListenerThread (TcpConnection listener, int port) {
         myListener = listener;
         myPort = port;
      }

      public void run() {
         myListener.waitForStream (myPort);
      }
   }

   /**
     * Start streaming replies over a TCP or UDP socket separate from the command/response channel.
     * @details TCP is preferred when data integrity is more important than latency. Eg. Data collection that must avoid missing/corrupt frames.
     * UDP is preferred when low latency is more important than high reliability of delivery. Eg. Data driving a real-time display.
     *
     * It is possible to interleave streaming replies on the same socket as commands and responses are processed.
     * However, this greatly complicates reply handling since the client must match replies to the commands
     * that generated them. It is much simpler to stream data to a separate port.
     *
     * @param cmd The command to stream, including its options. Eg. "TX 0801"
     * @param streamId A string used to uniquely identify a stream from other streams.
     * @param protocol Specifies the protocol (how the data is transmitted)
     * @param port Specifies the port that the client should listen on.
     */
   public int startStreaming (
      String cmd, String streamId, Protocol protocol, String port) {

      // Append parameters to the command. --protocol=TCP is the default, but it must be specified if UDP is desired
      String command = "STREAM" + (protocol == Protocol.TCP ? "" : " --protocol=UDP");

      // The command string is used as the streamId if the streamId is not provided
      if (!streamId.isEmpty()) {
         command += " --id=" + streamId;
      }

      // The port distinguishes the socket from other streams
      if (!port.isEmpty()) {
         command += " --port=" + port;
      }


      // Open the streaming connection
      Connection streamingConnection;
      String response = "";
      if (protocol == Protocol.UDP) {
         System.out.println ("Error: UDP protocal not yet supported");
         return 0x01; // invalid command error
      }
      else if (protocol == Protocol.TCP) {
         // Setup a socket to listen for incoming TCP connections
         TcpConnection tcpListener = new TcpConnection();
         StreamListenerThread listenerThread =
            new StreamListenerThread (tcpListener, Utils.stringToInt(port));
         listenerThread.start();

         // Send the STREAM command to start streaming replies
         sendCommand(command + " " + cmd);
         response = readResponse();

         // Wait for the thread to return once tcpListener has setup itself as the streaming socket
         try {
            listenerThread.join();
            streamingConnection = tcpListener;
            // Keep track of open streams
            streamMap.put(streamId, streamingConnection);
            
            // Return the error code from the STREAM command
            return getErrorCodeFromResponse(response);
         }
         catch (InterruptedException e) {
            return 0x05; // timed out error
         }
      }
      else {
         System.out.println ("Error: unknown protocal "+protocol);
         return 0x01; // invalid command error
      }

   }

   /*
    * Stop streaming using USTREAM
    */
   public int stopStreaming() {
      return stopStreaming ("");
   }

   /*
    * Stop streaming using USTREAM
    */
   public int stopStreaming(String streamId) {
      String command = "USTREAM";
      if (!streamId.isEmpty()) {
         // Try to find the given key in the map
         Connection connection = streamMap.get(streamId);
         if (connection == null) {
            return -1; // key not found
         }
         // Send USTREAM comamnd
         command += " --id=" + streamId;
         sendCommand(command);
         String response = readResponse();

         // Free resources allocated to the Connection, and remove it from the map
         connection.dispose();
         streamMap.remove (streamId);

         // Return the error code for the USTREAM
         return getErrorCodeFromResponse(response);
      }
      else {
         // Blank streamId means close all open streams
         boolean errorDetected = false;
         String response = "";
         for (Map.Entry<String,Connection> entry : streamMap.entrySet()) {
            // USTREAM each element in the map
            command = "USTREAM --id=" + entry.getKey();
            sendCommand(command);
            response = readResponse();

            // If any USTREAM has gone wrong, flag it
            errorDetected |= (getErrorCodeFromResponse(response) != 0);
            entry.getValue().dispose(); // free connection resources
         }

         // Remove all key-value pairs from the map
         streamMap.clear();

         // Return success(0) or error(-1) if any command had a problem
         return errorDetected ? -1 : 0;
      }
   }

   //! Read streaming replies from a socket
   public String readStream(String streamId) {
      
      Connection streamingConnection = streamMap.get(streamId);
      // Copy the response into the string
      String response = "";

      if (streamingConnection.getProtocol() == StreamingProtocol.TCP) {
         // Read from the device until we encounter a terminating carriage return (CR)
         StringBuilder sb = new StringBuilder();
         char lastChar = '\0';
         byte[] buf = new byte[1];
         while (lastChar != CR) {
            streamingConnection.read(buf, 1);
            lastChar = (char)(0xff & buf[0]);
            if (lastChar != CR) {
               sb.append (lastChar);
            }
         }
         response = sb.toString();
      }
      else if (streamingConnection.getProtocol() == StreamingProtocol.UDP) {
         System.out.println ("Error: UCP protocol not yet implemented");
         return "UDP not implemented";
      }

      int replyCRC16 = (int)Utils.hexStringToInt(substr(response,response.length()-4, 4));
      response = response.substring (0, response.length()-4); // strip CRC16 (4 chars)
      checkCRC (response, replyCRC16);

      // send response to terminal
      System.out.println ("<<" + response);
      return response;
   }

   /**
    * Retrieves tracking data as ASCII text using TX.
    * @param options An integer concatenated from TrackingReplyOption flags described in the API guide
    * @returns The result of the TX command: a list of handles and tracking data or an ERROR message.
    */
   public String getTrackingDataTX(short options) {

      // Send the TX command
      sendCommand("TX "+Utils.intToHexString(options, 4));
      return readResponse();
   }

   /**
    * Retrieves tracking data as ASCII text using TX and default options.
    * @returns The result of the TX command: a list of handles and tracking data or an ERROR message.
    */
   public String getTrackingDataTX() {
      return getTrackingDataTX (
         (short)(TrackingReplyOption.TransformData.getMask() |
                 TrackingReplyOption.AllTransforms.getMask()));
   }

   /**
    * Retrieves binary tracking data using BX and returns its string representation.
    * @param options An integer concatenated from TrackingReplyOption flags described in the API guide
    * @returns ToolData for all enabled tools, or an empty vector if an error occurred.
    */
   public ArrayList<ToolData> getTrackingDataBX(short options) {
      // Send the BX command
      sendCommand("BX "+Utils.intToHexString(options, 4));

      // Open a buffered reader on the connection to easily parse the binary reply
      BufferedReader reader = new BufferedReader(connection);
      reader.readBytes(6);
      short startSequence = reader.get_uint16();
      short replyLengthBytes = reader.get_uint16();

      // Verify the CRC16 of the header
      int headerCRC16 = (int)reader.get_uint16();
      if (!checkCRC (reader.getData(0, 4), 4, headerCRC16)) {
         return new ArrayList<>();
      }

      // In the case of an unexpected binary header, return an empty vector
      if (startSequence != START_SEQUENCE) {
         System.out.println ("Unrecognized start sequence: "+startSequence);
         return new ArrayList<>();
      }

      // Get all of the data once we know how many bytes to read: replyLengthBytes + 2 bytes for trailing CRC16
      reader.readBytes(replyLengthBytes + 2);

      // Verify the CRC16 of the data
      reader.skipBytes(replyLengthBytes);
      int dataCRC16 = reader.get_uint16();
      if (!checkCRC (reader.getData(6, replyLengthBytes), replyLengthBytes, dataCRC16)) {
         return new ArrayList<>();
      }
      reader.skipBytes(-replyLengthBytes -2); // move the BufferedReader's pointer back so we can parse the data

      // TODO: support all BX options. Just return if there are unexpected
      // options, we will be binary misaligned anyway.
      int validOptions = 
         (TrackingReplyOption.TransformData.getMask() |
          TrackingReplyOption.AllTransforms.getMask());
      if ((options & ~validOptions) != 0x0000) {
         System.out.println (
            "Reply parsing has not implemented options: " +
            Utils.intToHexString(options, 4));
         return new ArrayList<>();
      }

      ArrayList<ToolData> toolDataVector = new ArrayList<>();
      byte numHandles = reader.get_byte();
      for (int i = 0; i < numHandles; i++) {
         // Create a new ToolData
         ToolData toolData = new ToolData();
         toolDataVector.add(toolData);

         // From each two byte handle, extract the handle index and status
         toolData.transform.toolHandle = (short) reader.get_byte();
         byte handleStatus = reader.get_byte();

         // Parse BX 0001 - See API guide for protocol details
         if ((options & TrackingReplyOption.TransformData.getMask()) != 0) {
            // The transform is not transmitted at all if it is missing
            switch (handleStatus) {
               case 0x01: // Valid
                  toolData.transform.status = (short)Transform.Status.Enabled.ordinal();
                  toolData.transform.q0 = reader.get_double();
                  toolData.transform.qx = reader.get_double();
                  toolData.transform.qy = reader.get_double();
                  toolData.transform.qz = reader.get_double();
                  toolData.transform.tx = reader.get_double();
                  toolData.transform.ty = reader.get_double();
                  toolData.transform.tz = reader.get_double();
                  toolData.transform.error = reader.get_double();
                  break;
               case 0x04: // Disabled
                  // Disabled markers have no transform, status, or frame number
                  // don't return a ToolData for it, there's nothing there
                  toolDataVector.remove(toolDataVector.size()-1); 
                  continue;
               default:
                  // case 0x02: Missing or anything unexpected
                  // do nothing --> blank Transform object is already initialized as missing
                  break;
            };
              
            // Regardless of transform status, there is info about the port and frame
            toolData.portStatus = reader.get_uint32() & 0x0000FFFF;
            toolData.frameNumber = reader.get_uint32();
         }
      }

      // Add the systemStatus to each ToolData
      short systemStatus = reader.get_uint16();
      for (int t = 0; t < toolDataVector.size(); t++) {
         toolDataVector.get(t).systemStatus = systemStatus;
      }

      // Return the tool data
      return toolDataVector;
   }

   /**
    * Retrieves binary tracking data using BX2 and returns its string representation.
    * @param options A string containing the BX2 options described in the Vega API guide
    * @returns ToolData for all enabled tools that have new data since the last BX2, or an empty vector if an error occurred.
    */
   public ArrayList<ToolData> getTrackingDataBX2(String options) {
      // Send the BX2 command
      sendCommand("BX2 "+options);

      // Open a buffered reader on the connection to easily parse the binary reply
      BufferedReader reader = new BufferedReader(connection);

      // The BX2 reply begins with a 6 byte header:
      // (2-bytes) StartSequence: indicates how to parse the reply. A5C4 (normal)
      // (2-bytes) ReplyLength: length of the reply in bytes
      // (2-bytes) CRC16
      reader.readBytes(6);
      short startSequence = reader.get_uint16();
      short replyLengthBytes = reader.get_uint16();

      // Verify the CRC16 of the header
      int headerCRC16 = (int) reader.get_uint16();
      if (!checkCRC (reader.getData(0, 4), 4, headerCRC16)) {
         return new ArrayList<>();
      }

      // TODO: handle all BX2 reply types?
      if (startSequence != START_SEQUENCE) {
         // Return an empty vector if the binary response cannot be interpreted
         System.out.println (
            "Unrecognized BX2 reply header: " + startSequence +
            " - Not implemented yet!");
         return new ArrayList<>();
      }

      // Get all of the data once we know how many bytes to read: replyLengthBytes + 2 bytes for trailing CRC16
      reader.readBytes(replyLengthBytes + 2);

      // Verify the CRC16 of the data
      reader.skipBytes(replyLengthBytes);
      int dataCRC16 = reader.get_uint16();
      if (!checkCRC (reader.getData(6, replyLengthBytes), replyLengthBytes, dataCRC16)) {
         return new ArrayList<>();
      }
      reader.skipBytes(-replyLengthBytes -2); // move the BufferedReader's pointer back so we can parse the data

      // Parse the binary into meaningful objects
      GbfContainer container = new GbfContainer(reader);

      // Search the root GbfContainer to find the frame component
      ArrayList<ToolData> retVal = new ArrayList<>();
      for (int i = 0; i < container.components.size(); i++) {
         if (container.components.get(i).componentType ==
             GbfComponent.Type.Frame.ordinal()) {
            // Every GBF frame has GbfFrameDataItems for each type of tool:
            // Passive, ActiveWireless, Active
            GbfFrame frame = (GbfFrame)container.components.get(i);
            retVal = frame.getToolData();
            break;
         }
      }

      // If we didn't find any, then return an empty vector
      return retVal;
   }

   /**
    *  Gets the connection name.  For a TCP connection, this will be the IP4 address
    * @returns The connection name.
    */
   public String getConnectionName() {
      if (connection == null) {
         return null;
      }
      return connection.connectionName();
   }

   /**
    * Returns the error code of the response as a negative integer.
    */   
   public int getErrorCodeFromResponse(String response) {

      // Parse the error code from the response string and return it
      int errorCode = 0;
      if (response.startsWith("ERROR")) {
         errorCode = Utils.hexStringToInt(response.substring(5,7));
      }
      else if (response.startsWith("WARNING")) {
         errorCode = Utils.hexStringToInt(response.substring(7,9)) + WARNING_CODE_OFFSET;
      }
      // Use negative error codes to distinguish between port handles an errors.
      return errorCode * -1;
   }

   /**
    * Extracts the payload from the ASCII response by removing and checking CRC.
    * @returns The response as a String with trailing CR + CRC16 removed.
    */
   public String readResponse() {

      // Declare an empty string to hold the response
      StringBuilder response = new StringBuilder();
      byte[] bbuf = new byte[1];

      // Read from the device until we encounter a terminating carriage return (CR)
      while (bbuf[0] != CR) {
         if (connection.read(bbuf, 1) != 1) {
            throw new RuntimeException ("Lost connection");
         }
         response.append ((char)bbuf[0]);
      }

      // Trim trailing CR and verify the CRC16
      response.deleteCharAt (response.length()-1); // strip CR (1 char)
      int replyCRC16 = Utils.hexStringToInt(response.substring(response.length()-4, response.length()));
      response.delete(response.length()-4, response.length()); // strip CRC16 (4 chars)
      String responseStr = response.toString();
      checkCRC (responseStr, replyCRC16);
	
      // send response to terminal
      System.out.println ("<<"+responseStr);

      // Return whatever string the device responded with
      return responseStr;
   }

   private boolean checkCRC (String str, int check) {
      byte[] bytes = str.getBytes();
      return checkCRC (bytes, bytes.length, check);
   }

   private boolean checkCRC (byte[] bytes, int len, int check) {
      check &= 0xffff;
      int computedCrc = crcValidator.calculateCRC16(bytes, len);
      if (computedCrc != check) {
         System.out.printf (
            "CRC16 failed: computed 0x%x, received 0x%x\n", computedCrc, check);
         return false;
      }      
      else {
         return true;
      }
   }

   /**
    * Sends a command to the device.
    * @param command A string containing the ASCII command to send.
    * @returns The number of charaters written, or -1 if an error occurred.
    */
   public int sendCommand(String command) {

      // Display an error message if there is no open socket
      if (!connection.isConnected()) {
         System.out.println ("Cannot send command: "+command+"- No open socket!");
         return -1;
      }

      // Display the command that we're sending (except for BX, slows us down for real use)
      if (command.indexOf("BX") == -1) {
         System.out.println ("Sending command: "+command+" ...");
      }

      // Add CR character to command and write the command to the socket
      command += "\r";
      return connection.write(command.getBytes(), command.length());
   }

   //! Returns the human readable string corresponding to the given error or warning code.
   public String errorToString(int errorCode) {
      errorCode *= -1; // restore the errorCode to a positive value
      if (errorCode > WARNING_CODE_OFFSET) {
         return getWarningString(errorCode - WARNING_CODE_OFFSET);
      }
      else {
         return getErrorString(errorCode);
      }
   }

   /**
    * This method is used to lookup a human readable string when the device returns "WARNING[warnCode]"
    * @returns The message associated with the warning code, or "Warning code not found." if the code is unrecognized.
    */
   public String getWarningString (int warnCode) {
      if (warnCode < 0 || warnCode >= warningStrings.length) {
         return "Warning code not found.";
      }
      else {
         return warningStrings[warnCode];
      }
   }

   /**
    * This method is used to lookup a human readable string when the device returns "ERROR[errorCode]"
    * @returns The message associated with the error code, or "Error code not found." if the code is unrecognized.
    */
   public String getErrorString(int errorCode) {
      if (errorCode < 0 || errorCode >= errorStrings.length) {
         return "Error code not found.";
      }
      else {
         return errorStrings[errorCode];
      }
   }

   /**
    * Gets a user parameter using the GET command.
    * @param paramName The name of the user parameter.
    * @returns The result of the GET command in the format "[paramName]=[value]" or an ERROR message.
    */   
   public String getUserParameter(String paramName) {
      // Send the GET request
      sendCommand("GET "+paramName);
      // Return whatever the device responded with
      return readResponse();
   }

   /**
    * Sets a user parameter using the SET command.
    * @param paramName The name of the user parameter.
    * @param value The value to set.
    * @returns Zero for success, or the error code associated with the command.
    */
   public int setUserParameter(String paramName, String value)  {
      // Send the SET request
      sendCommand("SET "+paramName+"="+value);
      return getErrorCodeFromResponse(readResponse());
   }

   /**
    * Convenience method for substrings, more similar to the C++ syntax.
    */
   private String substr (String str, int off, int len) {
      return str.substring (off, off+len);
   }

   static String warningStrings[] = {
      
      "OKAY", // 0x0 not a warning
      "Possible hardware fault",
      "The tool violates unique geometry constraints",
      "The tool is incompatible with other loaded tools",
      "The tool is incompatible with other loaded tools and violate design contraints",
      "The tool does not specify a marker wavelength. The system will use the default wavelength."
   };

   static String errorStrings[] = {
      "OKAY", // 0x00 not an error
      "Invalid command.",
      "Command too long.",
      "Command too short.",
      "Invalid CRC calculated for command.",
      "Command timed out.",
      "Bad COMM settings.",
      "Incorrect number of parameters.",
      "Invalid port handle selected.",
      "Invalid priority.",
      "Invalid LED.",
      "Invalid LED state.",
      "Command is invalid while in the current mode.",
      "No tool is assigned to the selected port handle.",
      "Selected port handle not initialized.",
      "Selected port handle not enabled.",
      "System not initialized.", // 0x10
      "Unable to stop tracking.",
      "Unable to start tracking.",
      "Tool or SROM fault. Unable to initialize.",
      "Invalid Position Sensor characterization parameters.",
      "Unable to initialize the system.",
      "Unable to start Diagnostic mode.",
      "Unable to stop Diagnostic mode.",
      "Reserved",
      "Unable to read device's firmware version information.",
      "Internal system error.",
      "Reserved",
      "Invalid marker activation signature.",
      "Reserved",
      "Unable to read SROM device.",
      "Unable to write to SROM device.",
      "Reserved", // 0x20
      "Error performing current test on specified tool.",
      "Marker wavelength not supported.",
      "Command parameter is out of range.",
      "Unable to select volume.",
      "Unable to determine the system's supported features list.",
      "Reserved",
      "Reserved",
      "Too many tools are enabled.",
      "Reserved",
      "No memory is available for dynamic allocation.",
      "The requested port handle has not been allocated.",
      "The requested port handle is unoccupied.",
      "No more port handles available.",
      "Incompatible firmware versions.",
      "Invalid port description.",
      "Requested port is already assigned a port handle.", // 0x30
      "Reserved",
      "Invalid operation on the requested port handle.",
      "Feature unavailable.",
      "Parameter does not exist.",
      "Invalid value type.",
      "Parameter value is out of range.",
      "Parameter index out of range.",
      "Invalid parameter size.",
      "Permission denied.",
      "Reserved",
      "File not found.",
      "Error writing to file.",
      "Error removing file.",
      "Reserved",
      "Reserved",
      "Invalid or corrupted tool definition", // 0x40
      "Tool exceeds maximum markers, faces, or groups",
      "Required device not connected",
      "Reserved"
   };
}
