sawDATAQSerial
==============

cisst/SAW component to interface via USB (virtual COM port) to DATAQ DAQ (e.g. DI-145)

*Windows:*

 + tbb

*Linux:*

 + Linux automatically creates serial device, such as `ttyACM0`.
 + Check default permissions on `/dev/ttyACM0` using `ls -l /dev/ttyACM0`.  On Ubuntu a typical output should be:

   ```
   crw-rw---- 1 root dialout 166, 0 Jul 27 16:44 /dev/ttyACM0
   ```
 + Then add yourself to the `dialout` group or whatever group has read-write permissions on the tty.  Use `adduser <user_id> dialout`.  You need to logout and log back in for that change to be effective.  You can verify your user Id using the `id` command.
 + Example:

   ```
   tbd
   ```

*ROS:*
 + Package name is `dataq_serial_ros`, main node example is `dataq_serial_json`
 + The option `-t` starts the node in text only mode, otherwise the node has a Qt based GUI
 + The option `-n` allows to set the ROS namespace (default is `/dataq`).  This is useful if you have multiple DAQs
 + Example:

   ```
   tbd (see optoforce example)
   ```

*All:*

The software expects a JSON format configuration file, tbd.

Links
=====
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/

Dependencies
============
 * Linux, Mac OS, Windows, ...
 * cisst libraries: https://github.com/jhu-cisst/cisst
