rosrun srrg2_orazio orazio -gen -resource-path .
#we use the c preprocessor to assemble the final html
cpp -o index.html index.html.base
cpp -o orazio_joint0.html orazio_joint0.html.base
cpp -o orazio_joint1.html orazio_joint1.html.base
cpp -o orazio_joint2.html orazio_joint2.html.base
cpp -o orazio_joint3.html orazio_joint3.html.base
cpp -o orazio_system.html orazio_system.html.base
cpp -o orazio_drive.html orazio_drive.html.base
cpp -o orazio_servo.html orazio_servo.html.base
cpp -o orazio_imu.html orazio_imu.html.base
cpp -o orazio_sonar.html orazio_sonar.html.base
cpp -o orazio_joystick.html orazio_joystick.html.base

# we use sed to polish the cpp directives left in the files
sed -i '/^#/d' index.html
sed -i '/^#/d' orazio_joint0.html
sed -i '/^#/d' orazio_joint1.html
sed -i '/^#/d' orazio_joint2.html
sed -i '/^#/d' orazio_joint3.html
sed -i '/^#/d' orazio_system.html
sed -i '/^#/d' orazio_drive.html
sed -i '/^#/d' orazio_servo.html
sed -i '/^#/d' orazio_imu.html
sed -i '/^#/d' orazio_sonar.html
sed -i '/^#/d' orazio_joystick.html