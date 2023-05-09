
Simulator for curly hair. The library file isn't inlcuded but below are some (potentially outdated) instructions on dependencies. starter code is from assignment 1 of this class at CMU i found online. https://www.cs.cmu.edu/~scoros/cs15467-s16/ if forking, you can copy the lib file from the assignment listed or download them manually.
Windows:

First, determine the full path to the libs file in this repository,
for example:

    C:\Users\christoy\codebase\libs
    (targeting ..\libs)

Append this to your environment PATH variable. You can do this via
Control Panel > System > Advanced system settings > Environment Variables.
Find "Path" in your system variables, select it, and press edit. Go to
the very end of the value, add a semicolon, and paste in the path.
(Do not replace the entire value.)

Now download and install Visual Studio 2015 Community Edition from Dreamspark.
Once done, open SCP.sln. Visual Studio will prompt you to install some
dependencies, so install those, and open SCP.sln again.

After this, you should be able to compile and run the code. Note that running
the code in Release mode is MUCH faster than in Debug mode.

Linux:

This project has the following dependencies:

  - freetype
  - GLUT
  - GLEW
  - GLFW
  - AntTweakBar

The first three can be installed from most package managers; if using apt, the
recommended packages are libfreetype6-dev, glut3-dev, libglew-dev.

Version 3 of GLFW is required. At the moment, apt only has version 2, so
you will need to download glfw from the website (http://www.glfw.org/) and
build and install it from source. AntTweakBar should also be downloaded
and built and installed from source (http://anttweakbar.sourceforge.net/doc/).

