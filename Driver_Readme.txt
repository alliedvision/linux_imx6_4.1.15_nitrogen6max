I) short introduction and explanation of the dependecies between DTF Driver Module and kernel and static Driver instead of Driver Module.

Static driver:
================
*) Driver comes with kernel image and it will be loaded orderly.
*) For ex: If camera is needed to stream immediately after the bootup.

Module driver:
================
*) Driver will be loaded on demand. Some drivers can be loaded after booting (not like display) ex: camera, USB gadget drivers etc.,
*) The driver can be unloaded when ever its not required (it will free up the memory)
*) It will reduce the boot up time.

DTS: (Device tree source)
==========================
*) Using dts files, driver parameters (i2c address, memory size, data width, clock etc.,) can be passed to the kernel driver for initialization.
*) For ex: The driver needs to know which i2c address is used for the camera, so we can make use of dts file to mention the i2c address for the different cameras.
*) Using the dts file, we can change the camera parameters like lane count, csi clock, i2c slave address, auto negotiation of clock & lanes etc., depends on the driver dts parameters will be changed. 



II) how to do an installation of the CSI driver module at a standard kernel
Not possible.
We have changed lot of imx6 core driver code (as imx6 driver doesn't support most of the features but we enabled it) thus kernel symbol table is completely altered and we can't insert our camera driver in standard kernel, We can provide patch if needed for the customers.

III) how to enable and disable the driver module
*) Go to linux source tree
*) Type the below command.
make menuconfig ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
Device Drivers  --->
	<*> Multimedia support  --->
		[*]   V4L platform devices  --->
			<M> Allied Vision's camera support using mipi-csi2 for imx6 platform

*) Type 'y' to enable the driver and 'n' for disable. And 'm' for module driver.


IV) how to install the driver module
modprobe avt_imx6_csi2
OR
insmod <location>/avt_imx6_csi2.ko


V) how to switch on Debugging output of the driver
Before it's possible to switch the debugging output on it's necessary to deactivate the kernel moule if it's not already deactivated.
Deactivate the driver module with:
rmmod avt_imx6_csi2.ko
After this it is possible to activate it with the debug output flag:
modprobe avt_imx6_csi2 debug=1
OR
insmod <location>/avt_imx6_csi2.ko debug=1

Enabling the driver verbose messages at run time via sysfs:
To enable in core driver:
echo 1 > /sys/class/video4linux/videoX/debug_en  (where X indicates video device number which is allocated for our camera)
To disable in core driver:
echo 0 > /sys/class/video4linux/videoX/debug_en  (where X indicates video device number which is allocated for our camera)
Ex: echo 0 > /sys/class/video4linux/video3/debug_en

To enable in AVT camera driver:
echo 1 > /sys/bus/i2c/devices/X-00YY/debug_en  (where X indicates i2c bus number which is used by our camera and YY is our camera i2c slave address)
To disable in AVT camera driver:
echo 0 > /sys/bus/i2c/devices/X-00YY/debug_en  (where X indicates i2c bus number which is used by our camera and YY is our camera i2c slave address)
Ex: echo 0 > /sys/bus/i2c/devices/4-003c/debug_en  (Nitrogen6_MAX board)

VI) how to analyse the dmesg file regarding CSI driver.
Need to analyse the panic or kernel oops message (serial console terminal) whether the issues is from our camera driver or any other driver.
If its our driver then need to check the function and line no which causes the panic.
Ex:
Please refer to this link.
https://wiki.ubuntu.com/Kernel/KernelDebuggingTricks

VII) how to prepare a DTF (Device Tree File)
Please check the below doc.
linux-src/Documentation/devicetree/bindings/media/alliedvision-csi2.txt

VIII) how to compile a DTF
cd linux-src/
make nitrogen6x_alliedvision_defconfig ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
make dtbs -j4 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-

XI) how to install a DTF (including Linux requirements and possible system traps if there are some)
We should place the dtb files in /boot directory of SD card and u-boot would read the dtb file and copied into RAM.
