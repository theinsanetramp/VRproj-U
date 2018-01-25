To start program:
Power on raspberry pi using bottom power bank and plug into screen, keyboard and mouse.
Login to the pi, the username is pi and the password is raspberry.
Start the graphical desktop environment using the command:
   startx
Navigate to the folder CODE which is in the home directory of the pi user.
To build the project, type the command:
   make
To run the program using optical flow, type the command:
   sudo ./main -f
To run the program using camshift, type the command:
   sudo ./main -c

In places the code is a bit messy, in particular driving the motors due to a last minute change
where the Il Matto was used. Basically the pwm class only sets the GPIO to set the stepper
angle now, The speed is changed by writing to serial. This should be refactored to be within
the pwm class. Everything got a bit hectic with several modules deadlines so I never got round
to doing it.

If you have any questions about the project email toby_isaacs@live.co.uk
