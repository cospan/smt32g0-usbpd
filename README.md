# STM32G0 USBPD

STM32 based USBPD controller

At the moment this project is working with the [STM32G0 USB Type-C Power Delivery Discovery Kit](https://www.st.com/en/evaluation-tools/stm32g071b-disco.html) but I am planning to build a board that works with this controller to implement a 'Dual Role' USBPD Power Source/Sink

There is some documentation on how to get USBPD working but due to updates in the Cube software and the hardware differences within this discovery board the examples failed to work outside the box. Code from the [STM32CubeGo Git repo](https://github.com/STMicroelectronics/STM32CubeG0) from ST was used to get this working.

The major changes that were required include:

* Enabling various peripherals as can be seen within the cube file
* Copying over the following components from the STM32 Repo
	* BSP: `<STM32CubeG0 Git Repo>/Drivers/BSP/STM32G071B-Discovery* -> <Project Base>/Core/BSP`
	* Components: `<STM32CubeG0 Git Repo>/Drivers/BSP/Components -> <Project Base>/Core/Components`
	* Utilities: `<STM32CubeG0 Git Repo>/Utilities/Fonts -> <Project Base>/Core/Fonts`
* Modify `usbpd_dpm_user.c` file:
	* Function:
* Setup the `usbpd_pwr_user.c` to initialize the I2C driver to read the VBUS
	* Modify the function: `BSP_USBPD_PWR_VBUSInit` to call the initialize the I2C VBUS Controller using the function: `LL_BSP_PWR_VBUSInit`
	* Modify the function: `BSP_USBPD_PWR_VBUSGetVoltage` to read the voltage from the BSP using the function: `BSP_PWRMON_GetVoltage`
* Modify `main.c` to enable the MOSFETs that attach the CCx signals to the MCU


# TODO

* [X] Modify the policy of the design to select a contract other than 5V: 2020.12.01: Working @ 9V!
* [X] Demonstrate Writing to LCD: 2020.12.04: Demoed
* [ ] Fix the interface between the UCPD and the host to enable transmition of voltages and currents
* [ ] Turn on the system from USB-C Alone
* [ ] Modify the design to be a dual role port
* [ ] Demonstrate the design can be used to charge a battery
