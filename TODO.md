This is a list of things that need to be done for this project, from most important to least important.

================================
(DONE) Simplify Wifi connectivity
================================

================================
B4M API
================================
b4m_launch --b4m-ping
We are able to send a request to bike4mind, but can't get the response to come back, even though we can see it in the bike4mind webpage. Please see 'message_to_bike4mind_developers.md'

================================
Calibrate 'compass'
================================
b4m_launch --regression (--simulation)
When running the regression (real or sim) the robot overshoots the 360 degree spin by about 20 degrees. We should get this fixed in simulation first. Please use /regression/reference_screenshots_simulation/reference_initial.png as the reference for what the final position of the robot should be, since the robot starts at 0 degress and should end up at 360 degrees.



