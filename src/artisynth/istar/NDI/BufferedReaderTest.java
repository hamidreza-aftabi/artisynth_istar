package artisynth.istar.NDI;

import maspack.util.UnitTest;
import maspack.util.TestException;
import maspack.util.DynamicByteArray;
import java.util.*;

public class BufferedReaderTest extends UnitTest {

   TestConnection myTcon = new TestConnection();
   BufferedReader myReader = new BufferedReader (myTcon);

   private class TestConnection implements Connection {
      DynamicByteArray buffer = new DynamicByteArray();
      int idx;

      void pushByte (byte val) {
         buffer.add (val);
      }

      void pushShort (short val) {
         buffer.add ((byte)(val & 0xff));
         buffer.add ((byte)((val>>>8) & 0xff));
      }

      void pushInt (int val) {
         buffer.add ((byte)(val & 0xff));
         buffer.add ((byte)((val>>>8) & 0xff));
         buffer.add ((byte)((val>>>16) & 0xff));
         buffer.add ((byte)((val>>>24) & 0xff));
      }

      void pushFloat (float val) {
         pushInt (Float.floatToIntBits(val));
      }

      public int read (byte[] buf, int length) {
         for (int i=0; i<length; i++) {
            buf[i] = buffer.get(idx++);
         }
         return length;
      }
      
      // stub methods

      public void dispose() {
      }

      public boolean isConnected() {
         return true;
      }

      public boolean connect (String host) {
         return false;
      }

      public void disconnect() {
      }
      
      public String connectionName() {
         return "TestConnection";
      }

      public StreamingProtocol getProtocol() {
         return null;
      }

      public int write (byte[] buffer, int length) {
         return 0;
      }
   }

   void testByte (byte chk) {
      myTcon.pushByte (chk);
      myReader.readBytes (1);
      byte val = myReader.get_byte();
      if (val != chk) {
         throw new TestException ("read byte "+val+", expected "+chk);
      }
   }

   void testShort (short chk) {
      myTcon.pushShort (chk);
      myReader.readBytes (2);
      short val = myReader.get_uint16();
      if (val != chk) {
         throw new TestException ("read short "+val+", expected "+chk);
      }
   }

   void testInt (int chk) {
      myTcon.pushInt (chk);
      myReader.readBytes (4);
      int val = myReader.get_uint32();
      if (val != chk) {
         throw new TestException ("read int "+val+", expected "+chk);
      }
   }

   void testFloat (float chk) {
      myTcon.pushFloat (chk);
      myReader.readBytes (4);
      float val = (float)myReader.get_double();
      if (val != chk) {
         throw new TestException ("read float "+val+", expected "+chk);
      }
   }

   public void test() {
      testByte ((byte)0xff);
      testByte ((byte)0x07);
      testInt (0x3456789a);
      testInt (0xdeadbeef);
      testInt (0xff123456);
      testShort ((short)0xff78);
      testShort ((short)0x1234);
      testShort ((short)0xdead);
      testFloat (56.78e7f);
      testFloat (-1.6784e-6f);
   }
   
   public static void main (String[] args) {
      BufferedReaderTest tester = new BufferedReaderTest();
      tester.runtest();
   }

}
   
