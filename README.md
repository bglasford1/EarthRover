# EarthRover

If I could have picked my dream job it would have been to work on a planetary rover.  During my career the extra-terrestrial rovers were all going to Mars.  They included the Sojourner along with Spirit and Opportunity.  Since then there have been many more.  This is the culmination of many decades of attempts to complete this project that has always been bouncing around in my mind.  

My first attempt was a simple wooden chassis that was not well thought out.  My second attempt was built out of plexiglass and actually had wheel motors and a basic CPU made from a 6802 processor along with memory chips and various I/O chips.  Simple wheel movements were coded in assembly onto a ROM chip.  My old Radio Shack computer was used to burn the code onto the ROM.  

There is an older version of this document made in 2005 that was still based on the old 6802 processor chip set but with an off the shelf chassis.  In the last 15 years the technology has changed with regard to CPUs.  I have been creating projects with both the Arduino and the Raspberry Pi.  This update to the document is based on that new knowledge.

This rover is being designed as a multi-functional, expandable robot platform.  The rover shall be designed as an indoor/outdoor rover, i.e. an Earth rover based on the various Mars rovers. Some of the hazards include falling down stairs, running into objects and uneven surfaces and surface transitions.  Numerous sensors will be added to provide the maximum flexibility of present and future missions.  

The main interface is via a web page that sends commands to the rover and displays data, including a video feed.  The command interface is based on the Mars rover interface.  In other words, send a sequence of commands and let the rover execute those before analyzing the results and formulating a next command sequence.  
