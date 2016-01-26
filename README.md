# LogomaticV2

This software is a fork of Sparkfun's Logomatic software. It is designed specifically to
work with an Adafruit Ultimate GPS (9600 Baud), Adafruit ADXL335 triple-axis 
accelerometer and a TMP36 temperature sensor.

The point of this rewrite is to make the Logomatic suitable for data capture on a
High Altitude Balloon. GPS location and altitude combined with accelerometer and 
temperature readings will help understand the wind conditions aloft. The accelerometer
will also provide valuable information on the forces exerted on the payload during
landing.

Step #1: Clean up ADC logging (Completed 01/21/16)
	a. Put commas between each ADC reading
	b. Write the ADC pin status at the top of the logging file
		- Y = in use, N = not in use
		
Step #2: Attach Adafruit Ultimate GPS (01/24/16 - Waiting for connectors)
	a. Attach GPS to Logomatic using mini-clips (avoid soldering damage)
	b. Strip out interrupt driven ADC reading code
	c. Use UART Interrupt code to read GPS NMEA sentences
		- consider using '$' to trigger a sentence start and CR/LF to trigger
		  a sentence stop, then parse and save
	d. Parse RMC & GGA NMEA sentences to verify GPS validity and get date & time
	e. Save only RMC and GGA sentences
	
Step #3: Write code to include ADC readings (01/26/16 - not started)
	a. Trigger ADC read from CR/LF of NMEA sentences in Step #2
	b. Create separate buffers to hold ADC data
	c. Read ADC values
	d. Save GPS date & time, and ADC values