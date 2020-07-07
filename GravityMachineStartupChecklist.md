# Gravity Machine Start-up Checklist

Ensure Arduino has the appropriate firmware.
Launch the Tracking script.

- Check both cameras are triggering
- Hit "Start Tracking" and ensure the GUI is active and an object is tracked.
- Hit "Record" and check if images are being saved.
- Move the stages in "Manual Mode" and check if the 2 linear encoders and rotational encoders are functional.
	- Check the distance from end-to-end for the linear stages and see if they match the known stage travel.
	- Check the "virtual depth" by moving a quarter turn of the chamber and ensure that it is (pi/2)*(Radius)
- Hit "Homing" and ensure the stage can position itself and the limit switches are active.


