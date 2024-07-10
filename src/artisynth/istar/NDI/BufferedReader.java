package artisynth.istar.NDI;

import maspack.util.DynamicByteArray;
import java.util.*;

public class BufferedReader {
   
   Connection connection;
   int currentIndex;
   DynamicByteArray buffer;

   public BufferedReader (Connection connection) {
      this.connection = connection;
      buffer = new DynamicByteArray();
      currentIndex = 0;
   }

   String byteToHexString (byte input) {
      String str = Integer.toHexString (input);
      if (str.length() == 1) {
         str = "0" + str;
      }
      return str;
   }

   public String toString() {
      StringBuilder sb = new StringBuilder();
      for (int i=0; i<buffer.size(); i++) {
         sb.append (byteToHexString (buffer.get(i)));
         sb.append (' ');
      }
      return sb.toString();
   }

   byte[] getData(int start, int length) {
      
      byte[] retVal;
      if ((start + length) <= buffer.size()) {
         retVal = new byte[length];
         int k=0;
         for (int i = start; i < (start + length); i++) {
            retVal[k++] = buffer.get(i);
         }
      }
      else {
         retVal = new byte[0];
      }
      return retVal;
   }

   void readBytes(int numBytes) {
      
      byte[] bbuf = new byte[1];
      while (numBytes-- > 0) {
         connection.read(bbuf, 1);
         buffer.add(bbuf[0]);
      }
   }

   void skipBytes(int numBytes) {
      if (currentIndex + numBytes >= buffer.size()) {
         // Don't wander off the end of the buffer
         currentIndex = (int)buffer.size() - 1;
      }
      else if (currentIndex + numBytes < 0) {
         // Don't rewind beyond the first element
         currentIndex = 0;
      }
      else {
         // Move the index by the given number of bytes
         currentIndex += numBytes;
      }
   }

   byte get_byte() {
      return buffer.get(currentIndex++);
   }

   short get_uint16() {

      // The data is little-endian = least significant byte (LSB) in lowest memory address
      // Therefore, we need to construct the 16 bit integer by shifting and adding the two 8 bit bytes
      byte b0 = get_byte();
      byte b1 = get_byte();
      return (short)((0xff&b0) + ((0xff&b1)<<8));
   }

   int get_uint32() {

      // The data is little-endian = least significant byte (LSB) in lowest memory address
      // Therefore, we need to construct the 32 bit integer by shifting and adding the four 8 bit bytes
      byte b0 = get_byte();
      byte b1 = get_byte();
      byte b2 = get_byte();
      byte b3 = get_byte();
      return (0xff&b0) + ((0xff&b1)<<8) + ((0xff&b2)<<16) + ((0xff&b3)<<24);
   }

   double get_double() {

      // Read the little endian float and convert it to a double
      byte b0 = get_byte();
      byte b1 = get_byte();
      byte b2 = get_byte();
      byte b3 = get_byte();
      return (double)Float.intBitsToFloat (
         (0xff&b0) + ((0xff&b1)<<8) + ((0xff&b2)<<16) + ((0xff&b3)<<24));
   }

}
